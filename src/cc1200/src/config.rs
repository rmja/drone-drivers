use crate::opcode::{ExtReg, Reg};

pub struct Cc1200Config<'a> {
    pub first: Reg,
    pub values: &'a [u8],
    pub ext_first: ExtReg,
    pub ext_values: &'a [u8],
}

impl Cc1200Config<'_> {
    pub fn is_full(&self) -> bool {
        self.first == Reg::IOCFG3
            && Reg::IOCFG3 as usize + self.values.len() - 1 == Reg::PKT_LEN as usize
            && self.ext_first == ExtReg::IF_MIX_CFG
            && ExtReg::IF_MIX_CFG as usize + self.ext_values.len() - 1 == ExtReg::PA_CFG3 as usize
    }

    pub fn is_patch(&self) -> bool {
        !self.is_full()
    }

    pub fn reg(&self, reg: Reg) -> Option<u8> {
        let index = reg as usize - self.first as usize;
        self.values.get(index).map(|v| *v)
    }

    pub fn ext_reg(&self, reg: ExtReg) -> Option<u8> {
        let index = reg as usize - self.ext_first as usize;
        self.ext_values.get(index).map(|v| *v)
    }
}

pub enum Cc1200Gpio {
    Gpio0,
    Gpio1,
    Gpio2,
    Gpio3,
}
