use core::{cell::RefCell, cmp::min, mem, pin::Pin, sync::atomic::{AtomicU8, Ordering}, task::{Context, Poll}};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::prelude::*;

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi, RxFifoOverflowError, State, Strobe, TimeoutError, TxFifoUnderflowError, drv::{RX_FIFO_SIZE, TX_FIFO_SIZE}, opcode::{ExtReg, Reg}, regs::{FifoCfg, Mdmcfg1, PktCfg0, PktCfg1, PktCfg2, RfendCfg0, RfendCfg1}};

/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. De-asserted
/// when the RX FIFO is drained below (or is equal) to the same threshold.
const GPIO_CFG_RXFIFO_THR: u8 = 0;
/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR or the end of
/// packet is reached. De-asserted when the RX FIFO is empty.
const GPIO_CFG_RXFIFO_THR_PKT: u8 = 1;
/// Asserted when the TX FIFO is filled above (or is equal to)
/// (127−FIFO_CFG.FIFO_THR). De-asserted when the TX FIFO is drained below the
/// same threshold.
const GPIO_CFG_TXFIFO_THR: u8 = 2;
/// Asserted when sync word has been received and de-asserted at the end of the
/// packet. Will de-assert when the optional address and/or length check fails
/// or the RX FIFO overflows/underflows
const GPIO_CFG_PKT_SYNC_RXTX: u8 = 6;
/// Asserted when IDLE or RX, de-asserted when TX ot SETTLING.
const MARC_2PIN_STATUS_1: u8 = 37;

pub struct PacketController<
    Port: Cc1200Port,
    Spi: Cc1200Spi<A>,
    Chip: Cc1200Chip<A>,
    Tim: Cc1200Timer<A>,
    Upt: Cc1200Uptime<T, A>,
    Al: Alarm<T>,
    T: Tick,
    A,
> {
    driver: Arc<Cc1200Drv<Port, Al, T, A>>,
    spi: Arc<Mutex<Spi>>,
    chip: Arc<RefCell<Chip>>,
    timer: Tim,
    uptime: Arc<Upt>,
    config: &'static Cc1200Config<'static>,
    fifo_gpio: Cc1200Gpio,
    written_to_fifo: usize,
    write_queue: Vec<u8>,
}

#[derive(Debug)]
pub struct Packet<T: Tick> {
    /// The upstamp sampled right after the syncword has been detected.
    pub sof_upstamp: TimeSpan<T>,
    /// The average rssi for the packet.
    // pub rssi: Rssi,
    /// The received packet bytes.
    pub bytes: Vec<u8>,
}

struct Receive<Port: Cc1200Port, Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Upt: Cc1200Uptime<T, A>, Al: Alarm<T>, T: Tick, A> {
    state: AtomicU8,
    driver: Arc<Cc1200Drv<Port, Al, T, A>>,
    spi: Arc<Mutex<Spi>>,
    chip: Arc<RefCell<Chip>>,
    uptime: Arc<Upt>,
    config: &'static Cc1200Config<'static>,
    event_gpio_iocfg_reg: Reg,
    packet: Mutex<Option<Packet<T>>>,
}

#[derive(Copy, Clone, PartialEq)]
enum ReceiveState {
    WaitingForStartOfFrame,
    WaitingForEndOfFrame,
}

impl<
        Port: Cc1200Port,
        Spi: Cc1200Spi<A>,
        Chip: Cc1200Chip<A>,
        Tim: Cc1200Timer<A>,
        Upt: Cc1200Uptime<T, A>,
        Al: Alarm<T>,
        T: Tick,
        A,
    > PacketController<Port, Spi, Chip, Tim, Upt, Al, T, A>
{
    pub async fn setup(
        driver: Cc1200Drv<Port, Al, T, A>,
        spi: Arc<Mutex<Spi>>,
        chip: Chip,
        timer: Tim,
        uptime: Arc<Upt>,
        config: &'static Cc1200Config<'static>,
        fifo_gpio: Cc1200Gpio,
    ) -> Result<Self, TimeoutError> {
        assert!(config.is_full());
        assert_compatible_config(config);

        let mut ctrl = Self {
            driver: Arc::new(driver),
            spi,
            chip: Arc::new(RefCell::new(chip)),
            timer,
            uptime,
            config,
            fifo_gpio,
            written_to_fifo: 0,
            write_queue: Vec::new(),
        };

        ctrl.write_config(config).await;
        ctrl.idle().await;

        Ok(ctrl)
    }

    /// Patch the currently assigned configuration.
    pub async fn write_config<'a>(&mut self, config: &Cc1200Config<'a>) {
        assert_compatible_config(config);

        let mut spi = self.spi.lock().await;
        let mut chip = self.chip.borrow_mut();

        // Patch the configuration.
        self.driver.write_config(&mut *spi, &mut *chip, config).await;
    }
    
    /// Transition chip to idle state.
    async fn idle(&mut self) {
        let mut spi = self.spi.lock().await;
        let mut chip = self.chip.borrow_mut();
        self.driver
            .strobe_until_idle(&mut *spi, &mut *chip, Strobe::SIDLE)
            .await;
    }

    /// Prepare bytes for transmission.
    /// This transfers bytes to the chip tx buffer if there is room.
    /// Bytes that cannot fit in the TX fifo are buffered and sent to the tranceiver during transmission.
    pub async fn write(&mut self, buf: &[u8]) {
        // Append the buffer to the write queue.
        self.write_queue.extend_from_slice(buf);

        if self.written_to_fifo < TX_FIFO_SIZE {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            let len = min(self.write_queue.len(), TX_FIFO_SIZE - self.written_to_fifo);
            self.driver.write_fifo(&mut *spi, &mut *chip, &self.write_queue[..len]).await;
            self.write_queue.drain(0..len);
            self.written_to_fifo += len;
        }
    }

    /// Start packet transmission asap.
    /// The transmitter enters idle after the transmission completes.
    pub async fn transmit(&mut self) -> Result<(), TxFifoUnderflowError> {
        self.transmit_delayed(self.driver.alarm.counter(), TimeSpan::ZERO).await
    }

    /// Start packet transmission after a delay.
    /// The transmitter enters idle after the transmission completes.
    pub async fn transmit_delayed(&mut self, base: u32, delay: TimeSpan<T>) -> Result<(), TxFifoUnderflowError> {
        assert_ne!(0, self.written_to_fifo, "One must use write() prior to starting transmission");

        let pkt_len = self.written_to_fifo + self.write_queue.len();
        assert!(pkt_len <= 256);

        let timer_pin = self.timer.pin();
        let mut fifo_below_thr_stream = self.timer.falling_edge_capture_overwriting_stream(1);

        {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Setup fifo pin.
            let reg = self.fifo_gpio.iocfg_reg();
            self.driver.write_regs(&mut *spi, &mut *chip, reg, &[GPIO_CFG_TXFIFO_THR]).await;

            // Use fixed packet length mode.
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_CFG0, &[
                PktCfg0(self.config.reg(Reg::PKT_CFG0).unwrap()).write_length_config(0b00).0, // Fixed packet length mode.
            ]).await;

            // Set packet length.
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_LEN, &[(pkt_len % 256) as u8]).await;

            // Everything is setup. Wait for the fire time.
            self.driver.alarm.sleep_from(base, delay).await;

            // Start transmitter.
            self.driver.strobe(&mut *spi, &mut *chip, Strobe::STX).await;

            // Do not wait for calibration and settling.
        }

        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;
        while !self.write_queue.is_empty() {
            // Wait for fifo buffer to go below threshold.
            fifo_below_thr_stream.next().await;
            
            while !timer_pin.get() {
                let mut spi = self.spi.lock().await;
                let mut chip = self.chip.borrow_mut();

                let len = core::cmp::min(self.write_queue.len(), fifo_thr);
                self.driver.write_fifo(&mut *spi, &mut *chip, &self.write_queue[..len]).await;
                self.write_queue.drain(0..len);

                if self.driver.last_status().state() == State::TX_FIFO_ERROR {
                    // It seems that we came too late with the FIFO refill.
                    // Flush TX buffer.
                    self.driver
                        .strobe(&mut *spi, &mut *chip, Strobe::SFTX)
                        .await;

                    self.write_queue.clear();
                    self.written_to_fifo = 0;
                    return Err(TxFifoUnderflowError)
                }
            }
        }

        drop(fifo_below_thr_stream);

        {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Start listening for a rising edge
            // This cannot happen with the old pin configuration, as a rising
            // edge in that case only can happen when we write to the tx fifo.
            let mut tx_done_stream = self.timer.rising_edge_capture_overwriting_stream(1);

            // Re-define fifo pin to be asserted when idle.
            let reg = self.fifo_gpio.iocfg_reg();
            self.driver.write_regs(&mut *spi, &mut *chip, reg, &[MARC_2PIN_STATUS_1]).await;

            // See if transmission is already completed.
            if !timer_pin.get() {
                // Wait for transmission to complete.
                tx_done_stream.next().await;
            }
        }

        self.written_to_fifo = 0;

        Ok(())
    }

    /// Start receiver in fixed packet length mode.
    pub async fn receive_fixed_length<'a>(&'a mut self, length: usize) -> PacketStream<'a, T> {
        let timer_pin = self.timer.pin();

        // Start listen for interrupt events before we start the receiver.
        let event_stream = self.timer.rising_edge_capture_overwriting_stream(2);

        let receive = Arc::new(Receive {
            state: AtomicU8::new(0),
            driver: self.driver.clone(),
            spi: self.spi.clone(),
            chip: self.chip.clone(),
            uptime: self.uptime.clone(),
            config: self.config,
            event_gpio_iocfg_reg: self.fifo_gpio.iocfg_reg(),
            packet: Mutex::new(None),
        });

        // Make initial configuration and start the receiver
        {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Set packet length
            assert!(length <= 256);
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_LEN, &[
                (length % 256) as u8
            ]).await;

            // Use fixed packet length mode.
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_CFG0, &[
                PktCfg0(self.config.reg(Reg::PKT_CFG0).unwrap()).write_length_config(0b00).0, // Fixed packet length mode.
            ]).await;

            // Configure event stream to emit when start of frame is detected.
            receive.set_wait_for_sof(&mut *spi, &mut *chip).await;

            // Flush RX buffer
            self.driver
                .strobe(&mut *spi, &mut *chip, Strobe::SFRX)
                .await;

            // Start receiver.
            self.driver.strobe(&mut *spi, &mut *chip, Strobe::SRX).await;

            // Do not wait for calibration and settling.
        }
        
        let packet_stream = event_stream.then(move |capture| {
            let receive = receive.clone();
            let timer_pin = timer_pin.clone();
            async move {
                let mut spi = receive.spi.lock().await;
                let mut chip = receive.chip.borrow_mut();
                let mut current = receive.packet.try_lock().unwrap();
                let event_upstamp = receive.uptime.upstamp(capture);

                if current.is_none() {
                    // There is no current packet - we just detected a Start-Of-Frame.

                    receive.set_wait_for_fifo_or_eof(&mut *spi, &mut *chip).await;

                    *current = Some(Packet {
                        sof_upstamp: event_upstamp,
                        bytes: vec![]
                    });

                    None
                }
                else {
                    // We are currently receiving a packet...
                    let mut packet = current.take().unwrap();
                    let mut remaining = length + receive.fifo_append_status_bytes().then_some(2).unwrap_or_default() - packet.bytes.len();

                    // Read until data-ready goes low while we still haven't received the entire packet.
                    while timer_pin.get() && remaining > 0 {
                        let read_size = receive.driver.read_ext_reg(&mut *spi, &mut *chip, ExtReg::NUM_RXBYTES).await as usize;
                        let mut rx_buf = vec![0; read_size];
                        receive.driver.read_fifo(&mut *spi, &mut *chip, &mut rx_buf).await;

                        match receive.driver.last_status().state() {
                            State::RX => {
                                remaining -= rx_buf.len();
                                packet.bytes.append(&mut rx_buf);
                            }
                            state => {
                                panic!("Unrecoverable state {:?}", state);
                            }
                        }
                    }

                    if remaining == 0 {
                        if receive.fifo_append_status_bytes() {
                            // Remove the status bytes from the received packet bytes
                            packet.bytes.pop();
                            packet.bytes.pop();
                        }

                        // The packet is fully received, start waiting for a new packet.
                        receive.set_wait_for_sof(&mut *spi, &mut *chip).await;

                        Some(packet)
                    }
                    else {
                        // Packet is not yet fully received
                        *current = Some(packet);
                        None
                    }
                }
            }
        }).filter_map(|f| future::ready(f)); // Only use the events from the stream that actually produced a packet.

        PacketStream {
            stream: Box::pin(packet_stream),
        }
    }

    pub async fn receive<'a, F: 'a + Fn(&[u8]) -> Option<usize>>(&'a mut self, get_length: F) -> PacketStream<'a, T> {
        // Start listen for packet detected interrupts before the receiver is started.
        let timer_pin = self.timer.pin();
        let fifo_above_thr_stream = self.timer.rising_edge_capture_overwriting_stream(2);

        if let Some(length) = get_length(&[]) {
            // Fixed length mode
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Set pin to be asserted when receiving packet
            let reg = self.fifo_gpio.iocfg_reg();
            self.driver.write_regs(&mut *spi, &mut *chip, reg, &[GPIO_CFG_PKT_SYNC_RXTX]).await;

            // Use fixed length packet mode.
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_CFG0, &[
                PktCfg0(self.config.reg(Reg::PKT_CFG0).unwrap()).write_length_config(0b00).0, // Fixed packet length mode.
            ]).await;
        }
        else {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Set pin to be asserted when receiving packet
            let reg = self.fifo_gpio.iocfg_reg();
            self.driver.write_regs(&mut *spi, &mut *chip, reg, &[GPIO_CFG_PKT_SYNC_RXTX]).await;

            // Use infinite packet mode.
            self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_CFG0, &[
                PktCfg0(self.config.reg(Reg::PKT_CFG0).unwrap()).write_length_config(0b10).0, // Infinite packet length mode.
            ]).await;

            // Flush RX buffer
            self.driver
                .strobe(&mut *spi, &mut *chip, Strobe::SFRX)
                .await;

            // Start receiver.
            self.driver.strobe(&mut *spi, &mut *chip, Strobe::SRX).await;

            // Do not wait for calibration and settling.
        }

        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;

        const RX_STATE_WAITING_FOR_SOF: u8 = 0;
        const RX_STATE_RECEIVING_MIN_LENGTH: u8 = 1;
        const RX_STATE_RECEIVING_TAIL: u8 = 2;

        let uptime = self.uptime.clone();
        let spi = self.spi.clone();
        let chip = self.chip.clone();
        let driver = self.driver.clone();
        let rx_state = AtomicU8::new(RX_STATE_WAITING_FOR_SOF);
        let ongoing = Arc::new(Mutex::new(None));
        // let mut last_upstamp: Option<TimeSpan<T>> = None;
        // let mut remaining = 0;
        // let mut packet_bytes = vec![];

        let chunk_stream = fifo_above_thr_stream.then(move |capture| {
            let uptime = uptime.clone();
            let spi = spi.clone();
            let chip = chip.clone();
            let driver = driver.clone();
            let ongoing = ongoing.clone();
            let timer_pin = timer_pin.clone();
            async move {
                let mut spi = spi.lock().await;
                let mut chip = chip.borrow_mut();

                let mut ongoing = ongoing.try_lock().unwrap();

                if ongoing.is_none() { // Ok(RX_STATE_WAITING_FOR_SOF) == rx_state.compare_exchange(RX_STATE_WAITING_FOR_SOF, RX_STATE_RECEIVING_MIN_LENGTH, Ordering::AcqRel, Ordering::Relaxed) {
                    let upstamp = uptime.upstamp(capture);

                    
                    *ongoing = Some(Packet {
                        sof_upstamp: upstamp,
                        bytes: vec![]
                    });

                    // Set the FIFO threshold to match min_length
                    // self.driver.write_regs(&mut *spi, &mut *chip, Reg::FIFO_CFG, &[
                    //     FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).write_fifo_thr(min_length).0
                    // ]).await;

                    // Set pin to be asserted when FIFO has min_length bytes or the end of packet is reached.
                    // let reg = self.fifo_gpio.iocfg_reg();
                    // driver.write_regs(&mut *spi, &mut *chip, reg, &[GPIO_CFG_RXFIFO_THR_PKT]).await;

                    // None
                }

                None
                // else if Ok(RX_STATE_RECEIVING_MIN_LENGTH) == rx_state.compare_exchange(RX_STATE_RECEIVING_MIN_LENGTH, RX_STATE_RECEIVING_TAIL, Ordering::AcqRel, Ordering::Relaxed) {
                //     let mut rx_buf = vec![0; min_length as usize];
                //     driver.read_fifo(&mut *spi, &mut *chip, &mut rx_buf).await;

                //     let length = get_length(&rx_buf);
                //     assert!(length >= min_length as usize);
                //     remaining = length - min_length as usize;

                //     if remaining == 0 {
                //         Some(Packet {
                //             upstamp: last_upstamp.unwrap(),
                //             bytes: rx_buf,
                //         })
                //     }
                //     else {
                //         match driver.last_status().state() {
                //             State::RX => {
                //                 packet_bytes.reserve(length);
                //                 packet_bytes.extend_from_slice(&rx_buf);

                //                 // Reconfigure the FIFO threshold
                //                 // Set the FIFO threshold to use while receiving the remainder of the packet
                //                 self.driver.write_regs(&mut *spi, &mut *chip, Reg::FIFO_CFG, &[
                //                     fifo_thr as u8
                //                 ]).await;

                //                 // Set packet length.
                //                 self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_LEN, &[(length % 256) as u8]).await;
                //             }
                //             State::RX_FIFO_ERROR => {
                //                 // Flush RX buffer
                //                 driver.strobe(&mut *spi, &mut *chip, Strobe::SFRX).await;

                //                 // Restart receiver.
                //                 self.driver.strobe(&mut *spi, &mut *chip, Strobe::SRX).await;

                //                 rx_state.store(RX_STATE_WAITING_FOR_SOF, Ordering::Release);
                //             }
                //             state => {
                //                 panic!("Unrecoverable state {:?}", state);
                //             }
                //         }

                //         None
                //     }
                // }
                // else {
                //     // We have either received a FIFO block or the remaining of the packet
                //     if remaining <= fifo_thr {
                //         // This is the final receiption
                //         let mut rx_buf = vec![0; remaining];
                //         driver.read_fifo(&mut *spi, &mut *chip, &mut rx_buf).await;

                //         rx_state.store(RX_STATE_WAITING_FOR_SOF, Ordering::Release);

                //         Some(Packet {
                //             upstamp: TimeSpan::ZERO,
                //             bytes: packet_bytes,
                //         })
                //     }
                //     else {
                //         // This is just an intermediate FIFO block
                //         let mut rx_buf = vec![0; fifo_thr as usize];
                //         driver.read_fifo(&mut *spi, &mut *chip, &mut rx_buf).await;

                //         match driver.last_status().state() {
                //             State::RX => {
                //                 packet_bytes.extend_from_slice(&rx_buf);
                //                 remaining -= fifo_thr;
                //             }
                //             State::RX_FIFO_ERROR => {
                //                 // Flush RX buffer
                //                 driver.strobe(&mut *spi, &mut *chip, Strobe::SFRX).await;

                //                 // Restart receiver.
                //                 self.driver.strobe(&mut *spi, &mut *chip, Strobe::SRX).await;

                //                 rx_state.store(RX_STATE_WAITING_FOR_SOF, Ordering::Release);
                //             }
                //             state => {
                //                 panic!("Unrecoverable state {:?}", state);
                //             }
                //         }

                //         None
                //     }
                // }
            }
        }).filter_map(|f| future::ready(f));

        PacketStream {
            stream: Box::pin(chunk_stream),
        }
    }
}

impl<
        Port: Cc1200Port,
        Spi: Cc1200Spi<A>,
        Chip: Cc1200Chip<A>,
        Upt: Cc1200Uptime<T, A>,
        Al: Alarm<T>,
        T: Tick,
        A,
    > Receive<Port, Spi, Chip, Upt, Al, T, A> {

    fn state(&self) -> ReceiveState {
        unsafe { mem::transmute(self.state.load(Ordering::Acquire)) }
    }

    /// Get whether the two status bytes are appended to the packet in the rx fifo
    fn fifo_append_status_bytes(&self) -> bool {
        PktCfg1(self.config.reg(Reg::PKT_CFG1).unwrap()).append_status()
    }

    /// Transition to `ReceiveState::WaitingForStartOfFrame`.
    async fn set_wait_for_sof(&self, spi: &mut Spi, chip: &mut Chip) {
        // Set event pin to be asserted when syncword is detected (and deasserted when eof is detected)
        self.driver.write_regs(&mut *spi, &mut *chip, self.event_gpio_iocfg_reg, &[
            GPIO_CFG_PKT_SYNC_RXTX
        ]).await;

        self.state.store(ReceiveState::WaitingForStartOfFrame as u8, Ordering::Release);
    }

    /// Transition to `ReceiveState::WaitingForEndOfFrame`.
    async fn set_wait_for_fifo_or_eof(&self, spi: &mut Spi, chip: &mut Chip) {
        // Configure the event stream to emit when the fifo threshold is reached or the packet is fully received.
        self.driver.write_regs(&mut *spi, &mut *chip, self.event_gpio_iocfg_reg, &[GPIO_CFG_RXFIFO_THR_PKT]).await;

        self.state.store(ReceiveState::WaitingForEndOfFrame as u8, Ordering::Release);
    }
}

/// Ensure that a config is compatible with the controller operation.
fn assert_compatible_config<'a>(config: &Cc1200Config<'a>) {
    if let Some(val) = config.reg(Reg::MDMCFG1) {
        assert_eq!(val, Mdmcfg1(val).set_fifo_en().0, "FIFO must be enabled");
    }

    if let Some(val) = config.reg(Reg::PKT_CFG2) {
        assert_eq!(val, PktCfg2(val).write_pkt_format(0b00).0, "Packet mode must be Normal/FIFO mode");
    }

    if let Some(val) = config.reg(Reg::RFEND_CFG1) {
        assert_eq!(val, RfendCfg1(val).write_rxoff_mode(0b11).0, "Must re-enter RX when RX ends");
    }

    if let Some(val) = config.reg(Reg::RFEND_CFG0) {
        assert_eq!(val, RfendCfg0(val).write_txoff_mode(0b00).0, "Must enter IDLE after TX ends");
    }
}

pub struct PacketStream<'a, T: Tick> {
    stream: Pin<Box<dyn Stream<Item = Packet<T>> + 'a>>,
}

impl<T: Tick> Stream for PacketStream<'_, T> {
    type Item = Packet<T>;

    #[inline]
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.stream.as_mut().poll_next(cx)
    }
}

#[cfg(test)]
pub mod tests {
    use crate::configs::CC1200_WMBUS_MODECMTO_FULL;

    use super::*;

    #[test]
    pub fn configs_are_compatible() {
        assert_compatible_config(&CC1200_WMBUS_MODECMTO_FULL);
    }
}