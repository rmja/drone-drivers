use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick};
use futures::Future;

use crate::{Cc1200Chip, Cc1200Drv, Cc1200Port, Cc1200Spi, State, TimeoutError, opcode::{ExtReg, Opcode, Reg, Strobe}};

pub struct DebugController<
    Port: Cc1200Port,
    Spi: Cc1200Spi<A>,
    Chip: Cc1200Chip<A>,
    Al: Alarm<T>, T: Tick, A> {
    driver: Cc1200Drv<Port, Al, T, A>,
    spi: Arc<Mutex<Spi>>,
    chip: Chip,
}

impl<Port: Cc1200Port, Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Al: Alarm<T>, T: Tick, A> DebugController<Port, Spi, Chip, Al, T, A> {
    async fn setup(port: Port, alarm: Arc<Al>, spi: Arc<Mutex<Spi>>, chip: Chip) -> Result<Self, TimeoutError> {
        let mut ctrl = Self {
            driver: Cc1200Drv::init(port, alarm),
            spi,
            chip,
        };

        ctrl.driver.hw_reset(&mut ctrl.chip).await?;

        Ok(ctrl)
    }

    async fn idle(&mut self) {
        let mut spi = self.spi.try_lock().unwrap();
        self.driver.strobe_until(&mut *spi, &mut self.chip, Strobe::SIDLE, |status| status.state() == State::IDLE).await;
    }

    pub fn tx_unmodulated(&mut self) {
        // Enable custom frequency modulation

        let mdmcfg2 = 123u8;
        let mdmcfg1 = 123u8;
        let mdmcfg0 = 123u8;
        let pkt_cfg2 = 123u8;

        let batch = [
            Opcode::WriteSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::MDMCFG2 as u8,
            mdmcfg2,

            Opcode::WriteSingle(Reg::MDMCFG1).val(),
            mdmcfg1,

            Opcode::WriteSingle(Reg::MDMCFG0).val(),
            mdmcfg0,

            Opcode::WriteSingle(Reg::PKT_CFG2).val(),
            pkt_cfg2,

            Opcode::Strobe(Strobe::STX).val(),
        ];

        let mut spi = self.spi.try_lock().unwrap();
        // spi.write()

        // Calibration starts when entering TX.
    }
}