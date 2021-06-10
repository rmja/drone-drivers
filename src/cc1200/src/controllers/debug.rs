use alloc::sync::Arc;
use drone_core::sync::Mutex;
use drone_time::{Alarm, Tick};

use crate::{
    Cc1200Gpio,
    opcode::{ExtReg, Reg, Strobe},
    regs::{Mdmcfg0, Mdmcfg1, Mdmcfg2, PktCfg2},
    Cc1200Chip, Cc1200Config, Cc1200Drv, Cc1200Port, Cc1200Spi, State, TimeoutError,
};

pub struct DebugController<
    Port: Cc1200Port,
    Spi: Cc1200Spi<A>,
    Chip: Cc1200Chip<A>,
    Al: Alarm<T>,
    T: Tick,
    A,
> {
    driver: Cc1200Drv<Port, Al, T, A>,
    spi: Arc<Mutex<Spi>>,
    chip: Chip,
    config: &'static Cc1200Config<'static>,
}

impl<Port: Cc1200Port, Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Al: Alarm<T>, T: Tick, A>
    DebugController<Port, Spi, Chip, Al, T, A>
{
    pub async fn setup(
        driver: Cc1200Drv<Port, Al, T, A>,
        spi: Arc<Mutex<Spi>>,
        mut chip: Chip,
        config: &'static Cc1200Config<'static>,
    ) -> Result<Self, TimeoutError> {
        assert!(config.is_full());

        driver.hw_reset(&mut chip).await?;

        let ctrl = Self {
            driver,
            spi,
            chip,
            config,
        };

        Ok(ctrl)
    }

    pub async fn idle(&mut self) {
        let mut spi = self.spi.lock().await;
        self.driver
            .strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SIDLE)
            .await;
    }

    pub async fn tx_unmodulated(&mut self) {
        let mut spi = self.spi.lock().await;

        // Write default configuration.
        self.driver
            .write_config(&mut *spi, &mut self.chip, self.config)
            .await;

        self.driver
            .modify_ext_regs(&mut *spi, &mut self.chip, ExtReg::MDMCFG2, 1, |v| {
                v[0] = Mdmcfg2(v[0]).set_cfm_data_en().0; // Enable custom frequency modulation
            })
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::MDMCFG1, 2, |v| {
                v[0] = Mdmcfg1(v[0]).clear_fifo_en().0; // Disable FIFO (required by synchronous serial mode)
                v[1] = Mdmcfg0(v[1]).clear_transparent_mode_en().0; // Disable Transparent mode (required by synchronous serial mode)
            })
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::PKT_CFG2, 1, |v| {
                v[0] = PktCfg2(v[0]).write_pkt_format(0b01).0; // Synchronous serial mode
            })
            .await;

        if self.driver.last_status().state() == State::TX {
            self.driver
                .strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SIDLE)
                .await;
        }

        // Start transmitter
        self.driver
            .strobe(&mut *spi, &mut self.chip, Strobe::STX)
            .await;
    }

    pub async fn tx_modulated_01(&mut self) {
        let mut spi = self.spi.lock().await;

        // Write default configuration.
        self.driver
            .write_config(&mut *spi, &mut self.chip, self.config)
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::MDMCFG1, 2, |v| {
                v[0] = Mdmcfg1(v[0]).clear_fifo_en().0; // Disable FIFO (required by synchronous serial mode)
                v[1] = Mdmcfg0(v[1]).clear_transparent_mode_en().0; // Disable Transparent mode (required by synchronous serial mode)
            })
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::PKT_CFG2, 1, |v| {
                v[0] = PktCfg2(v[0]).write_pkt_format(0b01).0; // Synchronous serial mode
            })
            .await;

        // Set TXLAST != TXFIRST
        self.driver
            .write_ext_regs(&mut *spi, &mut self.chip, ExtReg::TXFIRST, &[0])
            .await;
        self.driver
            .write_ext_regs(&mut *spi, &mut self.chip, ExtReg::TXLAST, &[1])
            .await;

        if self.driver.last_status().state() == State::TX {
            self.driver
                .strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SIDLE)
                .await;
        }

        // Start transmitter
        self.driver
            .strobe(&mut *spi, &mut self.chip, Strobe::STX)
            .await;
    }

    pub async fn tx_modulated_pn9(&mut self) {
        let mut spi = self.spi.lock().await;

        // Write default configuration.
        self.driver
            .write_config(&mut *spi, &mut self.chip, self.config)
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::PKT_CFG2, 1, |v| {
                v[0] = PktCfg2(v[0]).write_pkt_format(0b10).0; // Random mode
            })
            .await;

        // Set TXLAST != TXFIRST (Required by random mode).
        self.driver
            .write_ext_regs(&mut *spi, &mut self.chip, ExtReg::TXFIRST, &[0])
            .await;
        self.driver
            .write_ext_regs(&mut *spi, &mut self.chip, ExtReg::TXLAST, &[1])
            .await;

        if self.driver.last_status().state() == State::TX {
            self.driver
                .strobe_until_idle(&mut *spi, &mut self.chip, Strobe::SIDLE)
                .await;
        }

        // Start transmitter.
        self.driver
            .strobe(&mut *spi, &mut self.chip, Strobe::STX)
            .await;
    }

    pub async fn rx(&mut self, data: Option<Cc1200Gpio>, clk: Option<Cc1200Gpio>) {
        // Enable custom frequency modulation
        let mut spi = self.spi.lock().await;

        // Write default configuration.
        self.driver
            .write_config(&mut *spi, &mut self.chip, self.config)
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::MDMCFG1, 2, |v| {
                v[0] = Mdmcfg1(v[0]).clear_fifo_en().0; // Disable FIFO (required by synchronous serial mode)
                v[1] = Mdmcfg0(v[1]).clear_transparent_mode_en().0; // Disable Transparent mode (required by synchronous serial mode)
            })
            .await;

        self.driver
            .modify_regs(&mut *spi, &mut self.chip, Reg::PKT_CFG2, 1, |v| {
                v[0] = PktCfg2(v[0]).write_pkt_format(0b01).0; // Synchronous serial mode
            })
            .await;

        if let Some(data) = data {
            let iocfg = match data {
                Cc1200Gpio::Gpio0 => Reg::IOCFG0,
                Cc1200Gpio::Gpio1 => Reg::IOCFG1,
                Cc1200Gpio::Gpio2 => Reg::IOCFG2,
                Cc1200Gpio::Gpio3 => Reg::IOCFG3,
            };
            self.driver
                .write_regs(&mut *spi, &mut self.chip, iocfg, &[0x09])
                .await;
        }

        if let Some(clk) = clk {
            let iocfg = match clk {
                Cc1200Gpio::Gpio0 => Reg::IOCFG0,
                Cc1200Gpio::Gpio1 => Reg::IOCFG1,
                Cc1200Gpio::Gpio2 => Reg::IOCFG2,
                Cc1200Gpio::Gpio3 => Reg::IOCFG3,
            };
            self.driver
                .write_regs(&mut *spi, &mut self.chip, iocfg, &[0x08])
                .await;
        }

        // Start receiver.
        self.driver
            .strobe(&mut *spi, &mut self.chip, Strobe::SRX)
            .await;
    }

    pub fn release(self) -> (Cc1200Drv<Port, Al, T, A>, Chip) {
        (self.driver, self.chip)
    }
}
