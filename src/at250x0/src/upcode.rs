#[allow(dead_code)]
pub(crate) enum Upcode {
    #[doc = "Set write enable latch"]
    WREN,
    #[doc = "Reset write enable latch"]
    WRDI,
    #[doc = "Read status register"]
    RDSR,
    #[doc = "Write status register"]
    WRSR,
    #[doc = "Read from memory array"]
    READ(u16),
    #[doc = "Write from memory array"]
    WRITE(u16),
}

impl Upcode {
    pub const fn val(self) -> u8 {
        const A8: u8 = 0b1000;
        match self {
            Upcode::WREN => 0b110,
            Upcode::WRDI => 0b100,
            Upcode::RDSR => 0b101,
            Upcode::WRSR => 0b001,
            Upcode::READ(addr) if addr > 0xFF => A8 | 0b011,
            Upcode::READ(_) => 0b011,
            Upcode::WRITE(addr) if addr > 0xFF => A8 | 0b010,
            Upcode::WRITE(_) => 0b010,
        }
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn read() {
        assert_eq!(0b0011, Upcode::READ(0x00).val());
        assert_eq!(0b0011, Upcode::READ(0xFF).val());
        assert_eq!(0b1011, Upcode::READ(0x100).val());
    }

    #[test]
    fn write() {
        assert_eq!(0b0010, Upcode::WRITE(0x00).val());
        assert_eq!(0b0010, Upcode::WRITE(0xFF).val());
        assert_eq!(0b1010, Upcode::WRITE(0x100).val());
    }
}
