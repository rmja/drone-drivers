use core::{cell::RefCell, cmp::min, pin::Pin, sync::atomic::{AtomicU8, Ordering}, task::{Context, Poll}};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::prelude::*;

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi, RxFifoOverflowError, State, TimeoutError, TxFifoUnderflowError, drv::TX_FIFO_SIZE, opcode::{Reg, Strobe}, regs::{FifoCfg, Mdmcfg1, PktCfg0, PktCfg1, PktCfg2, RfendCfg0, RfendCfg1}};

/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. De-asserted
/// when the RX FIFO is drained below (or is equal) to the same threshold.
const GPIO_CFG_RXFIFO_THR: u8 = 0;

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
}

#[derive(Debug)]
pub struct RxChunk<T: Tick> {
    /// The upstamp sampled when `fifo_thr` bytes has arrived in the CC1200 rx buffer.
    pub upstamp: TimeSpan<T>,
    /// The rssi sampled after `fifo_thr` bytes are in the rx buffer.
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
    pub async fn idle(&mut self) {
        let mut spi = self.spi.lock().await;
        let mut chip = self.chip.borrow_mut();
        self.driver
            .strobe_until_idle(&mut *spi, &mut *chip, Strobe::SIDLE)
            .await;
    }

    /// Start receiver in infinite packet mode reception.
    /// Note that the receiver is _not_ stopped when the stream is dropped, so idle() must be called manually after the stream is dropped.
    pub async fn receive<'a>(
        &'a mut self,
        capacity: usize,
    ) -> ChunkStream<'a, T> {
        let timer_pin = self.timer.pin();
        // Start listen for fifo interrupts before the receiver is started.
        // The capacity is 2 is that it allows for detecting of one rising edge while we are already draining the fifo.
        // That extra edge may happen multiple times in which case only the last on is actually executed.
        let fifo_above_thr_stream = self.timer.rising_edge_capture_overwriting_stream(2);

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

        let uptime = self.uptime.clone();
        let spi = self.spi.clone();
        let chip = self.chip.clone();
        let driver = self.driver.clone();
        let chunk_size = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize + 1; // There is one more byte in the RX FIFO than reported by the register value.
        let chunk_stream = fifo_above_thr_stream.then(move |capture| {
            let uptime = uptime.clone();
            let spi = spi.clone();
            let chip = chip.clone();
            let driver = driver.clone();
            let timer_pin = timer_pin.clone();
            async move {
                let upstamp = uptime.upstamp(capture);
                let mut spi = spi.lock().await;
                let mut chip = chip.borrow_mut();

                let mut results: Vec<Result<RxChunk<T>, RxFifoOverflowError>> = vec![];

                // Read until data-ready goes low.
                while timer_pin.get() {
                    let mut rx_buf = vec![0; chunk_size];
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
}


pub struct ChunkStream<'a, T: Tick> {
    stream: Pin<Box<dyn Stream<Item = Result<RxChunk<T>, RxFifoOverflowError>> + 'a>>,
}

impl<T: Tick> Stream for ChunkStream<'_, T> {
    type Item = Result<RxChunk<T>, RxFifoOverflowError>;

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