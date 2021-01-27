use crate::opcode::Reg;

#[derive(Copy, Clone)]
pub enum Cc1200Gpio {
    Gpio0,
    Gpio1,
    Gpio2,
    Gpio3,
}

impl Cc1200Gpio {
    pub fn iocfg_reg(self) -> Reg {
        match self {
            Cc1200Gpio::Gpio0 => Reg::IOCFG0,
            Cc1200Gpio::Gpio1 => Reg::IOCFG1,
            Cc1200Gpio::Gpio2 => Reg::IOCFG2,
            Cc1200Gpio::Gpio3 => Reg::IOCFG3,
        }
    }
}