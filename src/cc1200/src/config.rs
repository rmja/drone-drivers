use crate::opcode::{ExtReg, Reg};

pub struct Cc1200Config {
    pub first: Reg,
    pub values: &'static [u8],
    pub ext_first: ExtReg,
    pub ext_values: &'static [u8],
}