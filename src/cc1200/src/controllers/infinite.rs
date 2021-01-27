use core::{cell::RefCell, cmp::min, pin::Pin};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::{future::{Either, select}, prelude::*};

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi, State, TimeoutError, drv::TX_FIFO_SIZE, opcode::{Reg, Strobe}, regs::{FifoCfg, Mdmcfg0, Mdmcfg1, PktCfg0, PktCfg2, RfendCfg0, RfendCfg1}};

/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. De-asserted
/// when the RX FIFO is drained below (or is equal) to the same threshold.
const GPIO_CFG_RXFIFO_THR: u8 = 0;
/// Asserted when the TX FIFO is filled above (or is equal to)
/// (127−FIFO_CFG.FIFO_THR). De-asserted when the TX FIFO is drained below the
/// same threshold.
const GPIO_CFG_TXFIFO_THR: u8 = 2;
/// Asserted when IDLE or RX, de-asserted when TX ot SETTLING.
const MARC_2PIN_STATUS_1: u8 = 37;

#[derive(Debug)]
pub struct TxFifoUnderflowError;

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

        driver.hw_reset(&mut chip).await?;

        let this = Self {
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

        this.write_config().await;

        Ok(this)
    }

    async fn write_config(&self) {
        let mut spi = self.spi.try_lock().unwrap();
        let mut chip = self.chip.borrow_mut();

        // Write the full configuration.
        self.driver.write_config(&mut *spi, &mut *chip, self.config).await;

        // Write controller operational registers.
        self.driver.write_regs(&mut *spi, &mut *chip, Reg::MDMCFG1, &[
            Mdmcfg1(self.config.reg(Reg::MDMCFG1).unwrap()).set_fifo_en().0, // Enable FIFO.
        ]).await;

        self.driver.write_regs(&mut *spi, &mut *chip, Reg::PKT_CFG2, &[
            PktCfg2(self.config.reg(Reg::PKT_CFG2).unwrap()).write_pkt_format(0b00).0, // Normal/FIFO mode.
            self.config.reg(Reg::PKT_CFG1).unwrap(),
            PktCfg0(self.config.reg(Reg::PKT_CFG0).unwrap()).write_length_config(0b10).0, // Infinite packet length mode.
        ]).await;

        self.driver.write_regs(&mut *spi, &mut *chip, Reg::RFEND_CFG1, &[
            RfendCfg1(self.config.reg(Reg::RFEND_CFG1).unwrap()).write_rxoff_mode(0b11).0, // Re-enter RX when RX ends.
            RfendCfg0(self.config.reg(Reg::RFEND_CFG0).unwrap()).write_txoff_mode(0b00).0, // Enter IDLE after TX ends.
        ]).await;
    }

    /// Transition chip to idle state.
    pub async fn idle(&mut self) {
        let mut spi = self.spi.try_lock().unwrap();
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
            let mut spi = self.spi.try_lock().unwrap();
            let mut chip = self.chip.borrow_mut();

            let len = min(self.write_queue.len(), TX_FIFO_SIZE - self.written_to_fifo);
            self.driver.write_fifo(&mut *spi, &mut *chip, &self.write_queue[..len]).await;
            self.write_queue.drain(0..len);
            self.written_to_fifo += len;
        }
    }

    /// Start packet transmission.
    pub async fn transmit_packet(&mut self) -> Result<(), TxFifoUnderflowError> {
        assert_ne!(0, self.written_to_fifo, "One must use write() prior to starting transmission");

        let pkt_len = self.written_to_fifo + self.write_queue.len();
        assert!(pkt_len <= 256);

        let timer_pin = self.timer.pin();
        let mut fifo_below_thr_stream = self.timer.falling_edge_capture_overwriting_stream(1);

        {
            let mut spi = self.spi.try_lock().unwrap();
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

            // Start transmitter.
            self.driver.strobe(&mut *spi, &mut *chip, Strobe::STX).await;

            // Do not wait for calibration and settling.
        }

        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;
        while !self.write_queue.is_empty() {
            // Wait for fifo buffer to go below threshold.
            fifo_below_thr_stream.next().await;
            
            while !timer_pin.get() {
                let mut spi = self.spi.try_lock().unwrap();
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
            let mut spi = self.spi.try_lock().unwrap();
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
    ) -> Pin<Box<dyn Stream<Item = RxChunk<T>> + 'a>> {
        let timer_pin = self.timer.pin();
        let fifo_above_thr_stream = self.timer.rising_edge_capture_overwriting_stream(capacity);

        {
            let mut spi = self.spi.try_lock().unwrap();
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

        let uptime = self.uptime.clone();
        let spi = self.spi.clone();
        let chip = self.chip.clone();
        let driver = self.driver.clone();
        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;
        let chunk_stream = fifo_above_thr_stream.filter_map(move |capture| {
            let uptime = uptime.clone();
            let spi = spi.clone();
            let chip = chip.clone();
            let driver = driver.clone();
            let timer_pin = timer_pin.clone();
            async move {
                let upstamp = uptime.upstamp(capture);
                let mut rssi = None;
                let mut bytes = vec![];
                let mut spi = spi.try_lock().unwrap();
                let mut chip = chip.borrow_mut();

                // Read until data-ready goes low.
                while timer_pin.get() {
                    let mut rx_buf = vec![0; fifo_thr];
                    let chunk_rssi = driver
                        .read_rssi_and_fifo(&mut *spi, &mut *chip, &mut rx_buf)
                        .await;
                    if rssi.is_none() {
                        rssi = Some(chunk_rssi);
                    }

                    match driver.last_status().state() {
                        State::RX => {}
                        State::CALIBRATE => {}
                        State::SETTLING => {}
                        State::RX_FIFO_ERROR => {
                            // Flush RX buffer
                            driver.strobe(&mut *spi, &mut *chip, Strobe::SFRX).await;
                        }
                        state => {
                            panic!("Unrecoverable state {:?}", state);
                        }
                    }

                    bytes.append(&mut rx_buf);
                }

                rssi.map(|rssi| RxChunk {
                    upstamp,
                    rssi,
                    bytes,
                })
            }
        });

        Box::pin(chunk_stream)
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
