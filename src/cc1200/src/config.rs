use crate::opcode::{ExtReg, Reg};

pub struct Cc1200Config<'a> {
    pub first: Reg,
    pub values: &'a [u8],
    pub ext_first: ExtReg,
    pub ext_values: &'a [u8],
}

impl Cc1200Config<'_> {
    /// Get whether the configuration contains values for all registers.
    pub fn is_full(&self) -> bool {
        self.first == Reg::IOCFG3
            && Reg::IOCFG3 as usize + self.values.len() - 1 == Reg::PKT_LEN as usize
            && self.ext_first == ExtReg::IF_MIX_CFG
            && ExtReg::IF_MIX_CFG as usize + self.ext_values.len() - 1 == ExtReg::PA_CFG3 as usize
    }

    /// Get a register value, or None if the register is not part of the configuration.
    pub fn reg(&self, reg: Reg) -> Option<u8> {
        let index = reg as usize - self.first as usize;
        self.values.get(index).map(|v| *v)
    }

    /// Get an extended register value, or None if the register is not part of the configuration.
    pub fn ext_reg(&self, reg: ExtReg) -> Option<u8> {
        let index = reg as usize - self.ext_first as usize;
        self.ext_values.get(index).map(|v| *v)
    }
}