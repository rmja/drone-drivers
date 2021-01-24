use crate::opcode::{ExtReg, Reg};

pub struct Cc1200Config<'a> {
    pub first: Reg,
    pub values: &'a [u8],
    pub ext_first: ExtReg,
    pub ext_values: &'a [u8],
}

pub enum Cc1200Gpio {
    Gpio0,
    Gpio1,
    Gpio2,
    Gpio3,
}