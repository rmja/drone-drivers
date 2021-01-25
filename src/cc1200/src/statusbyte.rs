use core::mem::transmute;
use drone_core::bitfield::Bitfield;

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved(r, 0, 4),
    state_bits(r, 4, 3, "Indicates the current main state machine mode."),
    chip_rdy(r, 7, 1, "Stays high until power and crystal have stabilized. Should always be low when using the SPI interface."),
)]
pub struct StatusByte(pub u8);

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum State {
    IDLE = 0b000,
    RX = 0b001,
    TX = 0b010,
    FSTXON = 0b011,
    CALIBRATE = 0b100,
    SETTLING = 0b101,
    RX_FIFO_ERROR = 0b110,
    TX_FIFO_ERROR = 0b111,
}

impl StatusByte {
    pub fn state(self) -> State {
        unsafe { transmute(self.state_bits()) }
    }
}
