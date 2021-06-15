use core::{cell::RefCell, cmp::min, pin::Pin, task::{Context, Poll}};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::prelude::*;

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi, RxFifoOverflowError, State, Strobe, TimeoutError, TxFifoUnderflowError, drv::TX_FIFO_SIZE, opcode::Reg, regs::{FifoCfg, Mdmcfg1, PktCfg0, PktCfg2, RfendCfg0, RfendCfg1}};

/// Asserted when the TX FIFO is filled above (or is equal to)
/// (127−FIFO_CFG.FIFO_THR). De-asserted when the TX FIFO is drained below the
/// same threshold.
const GPIO_CFG_TXFIFO_THR: u8 = 2;
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
    pub upstamp: TimeSpan<T>,
    /// The average rssi for the packet.
    pub rssi: Rssi,
    /// The received packet bytes.
    pub bytes: Vec<u8>,
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

    pub async fn receive<'a, F: Fn(&[u8]) -> Option<usize>>(&'a mut self, min_length: usize, get_length: F) -> PacketStream<'a, T> {
        // Start listen for packet detected interrupts before the receiver is started.
        let fifo_above_thr_stream = self.timer.rising_edge_capture_overwriting_stream(2);

        let fixed_length_mode = Some(min_length) == get_length(&[]);

        if fixed_length_mode {
            todo!()
        }
        else {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Setup fifo pin.
            // let reg = self.fifo_gpio.iocfg_reg();
            // self.driver.write_regs(&mut *spi, &mut *chip, reg, &[min_length]).await;

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



        todo!();
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
    stream: Pin<Box<dyn Stream<Item = Result<Packet<T>, RxFifoOverflowError>> + 'a>>,
}

impl<T: Tick> Stream for PacketStream<'_, T> {
    type Item = Result<Packet<T>, RxFifoOverflowError>;

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