use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick};
use futures::prelude::*;

use crate::{Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, State, TimeoutError, opcode::{Reg, Strobe}, regs::FifoCfg};

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
    driver: Cc1200Drv<Port, Al, T, A>,
    spi: Arc<Mutex<Spi>>,
    chip: Chip,
    timer: Tim,
    uptime: Arc<Upt>,
    rx_buf: Vec<u8>,
    config: &'static Cc1200Config<'static>,
}

impl<Port: Cc1200Port, Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Tim: Cc1200Timer<A>, Upt: Cc1200Uptime<T, A>, Al: Alarm<T>, T: Tick, A>
InfiniteController<Port, Spi, Chip, Tim, Upt, Al, T, A> {
    pub async fn setup(
        driver: Cc1200Drv<Port, Al, T, A>,
        spi: Arc<Mutex<Spi>>,
        chip: Chip,
        timer: Tim,
        uptime: Arc<Upt>,
        config: &'static Cc1200Config<'static>,
    ) -> Result<Self, TimeoutError> {
        assert!(config.is_full());

        let fifo_thr = FifoCfg(config.reg(Reg::FIFO_CFG).unwrap()).fifo_thr();
        let rx_buf = vec![0; fifo_thr as usize];
        let mut ctrl = Self {
            driver,
            spi,
            chip,
            timer,
            uptime,
            rx_buf,
            config,
        };

        ctrl.driver.hw_reset(&mut ctrl.chip).await?;

        let mut spi = ctrl.spi.try_lock().unwrap();
        ctrl.driver
            .write_config(&mut *spi, &mut ctrl.chip, ctrl.config)
            .await;
        drop(spi);

        Ok(ctrl)
    }

    pub async fn idle(&mut self) {
        let mut spi = self.spi.try_lock().unwrap();
        self.driver.strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SIDLE).await;
    }

    pub async fn rx_stream(&mut self, capacity: usize) {
        let mut spi = self.spi.try_lock().unwrap();

        // Flush RX buffer
        self.driver.strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SFRX).await;

        self.timer.clear_pending_capture();
        let mut capture_stream = self.timer.capture_overwriting_stream(capacity);

        // Start receiver.
        self.driver.strobe(&mut *spi, &mut self.chip, Strobe::SRX).await;
        // Do not wait for calibration and settling.
        
        drop(spi);
        
        while let Some(capture) = capture_stream.next().await {
            let upstamp = self.uptime.upstamp(capture);

            let mut spi = self.spi.try_lock().unwrap();
            let rssi = self.driver.read_rssi_and_fifo(&mut *spi, &mut self.chip, &mut self.rx_buf).await;

            // if capture_stream.get() {
                
            // }

            match self.driver.last_status().state() {
                State::RX => {},
                State::CALIBRATE => {},
                State::SETTLING => {},
                State::RX_FIFO_ERROR => {
                    // Flush RX buffer
                    self.driver.strobe(&mut *spi, &mut self.chip, Strobe::SFRX).await;
                },
                state => {
                    panic!("Unrecoverable state {:?}", state);
                }
            }
        }
    }

    pub fn release(self) -> Cc1200Drv<Port, Al, T, A> {
        self.driver
    }
}