use core::{cell::RefCell, pin::Pin};

use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::prelude::*;

use crate::{
    opcode::{Reg, Strobe},
    regs::FifoCfg,
    Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, Rssi,
    State, TimeoutError,
};

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
    ) -> Result<Self, TimeoutError> {
        assert!(config.is_full());

        driver.hw_reset(&mut chip).await?;

        let mut spi_sess = spi.try_lock().unwrap();
        driver.write_config(&mut *spi_sess, &mut chip, config).await;
        drop(spi_sess);

        Ok(Self {
            driver: Arc::new(driver),
            spi,
            chip: Arc::new(RefCell::new(chip)),
            timer,
            uptime,
            config,
        })
    }

    pub async fn idle(&mut self) {
        let mut spi = self.spi.try_lock().unwrap();
        let mut chip = self.chip.borrow_mut();
        self.driver
            .strobe_until_idle(&mut *spi, &mut *chip, Strobe::SIDLE)
            .await;
    }

    pub async fn rx_stream<'a>(
        &'a mut self,
        capacity: usize,
    ) -> Pin<Box<dyn Stream<Item = RxChunk<T>> + 'a>> {
        let mut spi = self.spi.try_lock().unwrap();
        let mut chip = self.chip.borrow_mut();

        // Flush RX buffer
        self.driver
            .strobe_until_idle(&mut *spi, &mut *chip, Strobe::SFRX)
            .await;

        let timer_pin = self.timer.pin();
        let capture_stream = self.timer.capture_overwriting_stream(capacity);

        // Start receiver.
        self.driver.strobe(&mut *spi, &mut *chip, Strobe::SRX).await;
        // Do not wait for calibration and settling.

        drop(spi);

        let uptime = self.uptime.clone();
        let spi = self.spi.clone();
        let chip = self.chip.clone();
        let driver = self.driver.clone();
        let fifo_thr = FifoCfg(self.config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr() as usize;
        let chunk_stream = capture_stream.filter_map(move |capture| {
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
