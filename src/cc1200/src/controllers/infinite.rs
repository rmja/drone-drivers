use core::{cell::RefCell, cmp::min, pin::Pin, sync::atomic::{AtomicBool, Ordering}, task::{Context, Poll}};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::prelude::*;

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi, RxFifoOverflowError, State, TimeoutError, TxFifoUnderflowError, drv::TX_FIFO_SIZE, opcode::{Reg, Strobe}, regs::{FifoCfg, Mdmcfg1, PktCfg0, PktCfg2, RfendCfg0, RfendCfg1}};

/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. De-asserted
/// when the RX FIFO is drained below (or is equal) to the same threshold.
const GPIO_CFG_RXFIFO_THR: u8 = 0;
/// Asserted when the TX FIFO is filled above (or is equal to)
/// (127−FIFO_CFG.FIFO_THR). De-asserted when the TX FIFO is drained below the
/// same threshold.
const GPIO_CFG_TXFIFO_THR: u8 = 2;
/// Asserted when IDLE or RX, de-asserted when TX ot SETTLING.
const MARC_2PIN_STATUS_1: u8 = 37;

pub struct InfiniteController<
    Port: Cc1200Port,
    Spi: Cc1200Spi<A>,
    Chip: Cc1200Chip<A>,
    Tim: Cc1200Timer<A>,
    Upt: Cc1200Uptime<T, A>,
    Al: Alarm<T>,
    T: Tick,
    A,
> {
    receiving: Arc<AtomicBool>,
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
pub struct RxChunk<T: Tick> {
    /// The upstamp sampled after `fifo_thr` bytes.
    pub upstamp: TimeSpan<T>,
    /// The rssi sampled after `fifo_thr` bytes.
    pub rssi: Rssi,
    /// The received bytes, always a multiple of `fifo_thr`.
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
    > InfiniteController<Port, Spi, Chip, Tim, Upt, Al, T, A>
{
    pub async fn setup(
        driver: Cc1200Drv<Port, Al, T, A>,
        spi: Arc<Mutex<Spi>>,
        mut chip: Chip,
        timer: Tim,
        uptime: Arc<Upt>,
        config: &'static Cc1200Config<'static>,
        fifo_gpio: Cc1200Gpio,
    ) -> Result<Self, TimeoutError> {
        assert!(config.is_full());
        Self::assert_compatible_config(config);

        driver.hw_reset(&mut chip).await?;

        let mut this = Self {
            receiving: Arc::new(AtomicBool::new(false)),
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

        this.write_config(config).await;

        Ok(this)
    }

    /// Ensure that a config is compatible with the controller operation.
    fn assert_compatible_config<'a>(config: &Cc1200Config<'a>) {
        if let Some(val) = config.reg(Reg::MDMCFG1) {
            assert_eq!(val, Mdmcfg1(val).set_fifo_en().0); // FIFO must be enabled.
        }

        if let Some(val) = config.reg(Reg::PKT_CFG2) {
            assert_eq!(val, PktCfg2(val).write_pkt_format(0b00).0); // Packet mode must be Normal/FIFO mode.
        }

        if let Some(val) = config.reg(Reg::RFEND_CFG1) {
            assert_eq!(val, RfendCfg1(val).write_rxoff_mode(0b11).0); // Must re-enter RX when RX ends.
        }

        if let Some(val) = config.reg(Reg::RFEND_CFG0) {
            assert_eq!(val, RfendCfg0(val).write_txoff_mode(0b11).0); // Must enter IDLE after TX ends.
        }
    }

    /// Patch the currently assigned configuration.
    pub async fn write_config<'a>(&mut self, config: &Cc1200Config<'a>) {
        Self::assert_compatible_config(config);

        let mut spi = self.spi.lock().await;
        let mut chip = self.chip.borrow_mut();

        // Patch the configuration.
        self.driver.write_config(&mut *spi, &mut *chip, config).await;
    }

    /// Transition chip to idle state.
    pub async fn idle(&mut self) {
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
    pub async fn transmit_packet(&mut self) -> Result<(), TxFifoUnderflowError> {
        self.transmit_packet_delayed(self.driver.alarm.counter(), TimeSpan::ZERO).await
    }

    /// Start packet transmission after a delay.
    pub async fn transmit_packet_delayed(&mut self, base: u32, delay: TimeSpan<T>) -> Result<(), TxFifoUnderflowError> {
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

    /// Start receiver in infinite packet mode reception.
    pub async fn receive_stream<'a>(
        &'a mut self,
        capacity: usize,
    ) -> ChunkStream<'a, T> {
        let timer_pin = self.timer.pin();
        let fifo_above_thr_stream = self.timer.rising_edge_capture_overwriting_stream(capacity);

        {
            let mut spi = self.spi.lock().await;
            let mut chip = self.chip.borrow_mut();

            // Setup fifo pin.
            let reg = self.fifo_gpio.iocfg_reg();
            self.driver.write_regs(&mut *spi, &mut *chip, reg, &[GPIO_CFG_RXFIFO_THR]).await;

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

        self.receiving.store(true, Ordering::Relaxed);

        let uptime = self.uptime.clone();
        let spi = self.spi.clone();
        let chip = self.chip.clone();
        let driver = self.driver.clone();
        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;
        let chunk_stream = fifo_above_thr_stream.then(move |capture| {
            let uptime = uptime.clone();
            let spi = spi.clone();
            let chip = chip.clone();
            let driver = driver.clone();
            let timer_pin = timer_pin.clone();
            async move {
                let upstamp = uptime.upstamp(capture);
                let mut spi = spi.try_lock().unwrap();
                let mut chip = chip.borrow_mut();

                let mut results: Vec<Result<RxChunk<T>, RxFifoOverflowError>> = vec![];

                // Read until data-ready goes low.
                while timer_pin.get() {
                    let mut rx_buf = vec![0; fifo_thr];
                    let rssi = driver
                        .read_rssi_and_fifo(&mut *spi, &mut *chip, &mut rx_buf)
                        .await;

                    match driver.last_status().state() {
                        State::RX => {
                            results.push(Ok(RxChunk {
                                upstamp,
                                rssi,
                                bytes: rx_buf,
                            }));
                        }
                        State::CALIBRATE => {}
                        State::SETTLING => {}
                        State::RX_FIFO_ERROR => {
                            results.push(Err(RxFifoOverflowError));

                            // Flush RX buffer
                            driver.strobe(&mut *spi, &mut *chip, Strobe::SFRX).await;
                        }
                        state => {
                            panic!("Unrecoverable state {:?}", state);
                        }
                    }
                }

                let s = stream::iter(results);

                s
            }
        }).flatten();

        ChunkStream {
            receiving: self.receiving.clone(),
            stream: Box::pin(chunk_stream),
        }
    }

    pub fn release(self) -> (Cc1200Drv<Port, Al, T, A>, Chip, Tim) {
        let drv = Arc::try_unwrap(self.driver)
            .map_err(|_| ())
            .expect("Unable to unwrap driver");
        let chip = Arc::try_unwrap(self.chip)
            .map_err(|_| ())
            .expect("Unable to unwrap chip");

        (drv, chip.into_inner(), self.timer)
    }
}


pub struct ChunkStream<'a, T: Tick> {
    receiving: Arc<AtomicBool>,
    stream: Pin<Box<dyn Stream<Item = Result<RxChunk<T>, RxFifoOverflowError>> + 'a>>,
}

impl<T: Tick> Stream for ChunkStream<'_, T> {
    type Item = Result<RxChunk<T>, RxFifoOverflowError>;

    #[inline]
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.stream.as_mut().poll_next(cx)
    }
}

impl<T: Tick> Drop for ChunkStream<'_, T> {
    fn drop(&mut self) {
        assert!(!self.receiving.load(Ordering::Relaxed), "One must make sure to invoke idle() prior to dropping receive stream");
    }
}