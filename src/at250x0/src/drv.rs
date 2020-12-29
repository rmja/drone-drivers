use crate::{aligned_chunks::SliceExt, opcode::Opcode, At250x0Chip, At250x0Spi, At250x0Timer};
use core::{cell::RefCell, marker::PhantomData};
use drone_core::bitfield::Bitfield;

const PAGE_SIZE: usize = 8;
const INITIAL_TIMEOUT_US: u32 = 3000; // Wait at least 3 ms
const RETRY_INTERVAL_US: u32 = 100;

#[derive(Clone, Copy)]
pub enum At250x0Kind {
    At25010,
    At25020,
    At25040,
    At25010b,
    At25020b,
    At25040b,
}

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    bsy(r, 0, 1, "Ready/busy status"),
    wel(r, 1, 1, "Write enable latch"),
    bp(r, 2, 2, "Block write protection"),
    rfu(r, 4, 4, "Reserved for future use")
)]
struct StatusRegister(u8);

#[derive(Debug)]
pub struct WriteProtectError;

pub struct At250x0Drv<Timer: At250x0Timer<A>, A> {
    kind: At250x0Kind,
    timer: RefCell<Timer>,
    adapters: PhantomData<A>,
}

impl<Timer: At250x0Timer<A>, A> At250x0Drv<Timer, A> {
    pub fn new(kind: At250x0Kind, timer: Timer) -> Self {
        Self {
            kind,
            timer: RefCell::new(timer),
            adapters: PhantomData,
        }
    }

    /// Read a sequence of bytes from the EEPROM.
    pub async fn read<Spi, Chip>(&self, spi: &mut Spi, chip: &mut Chip, origin: u16, buf: &mut [u8])
    where
        Spi: At250x0Spi<A>,
        Chip: At250x0Chip<A>,
    {
        assert!(origin + buf.len() as u16 <= capacity(self.kind));

        chip.select();
        spi.write(&[Opcode::READ(origin).val(), (origin & 0xFF) as u8])
            .await;
        spi.read(buf).await;
        chip.deselect();
    }

    /// Write a sequence of bytes to the EEPROM.
    pub async fn write<Spi, Chip>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        origin: u16,
        buf: &[u8],
    ) -> Result<(), WriteProtectError>
    where
        Spi: At250x0Spi<A>,
        Chip: At250x0Chip<A>,
    {
        assert!(origin + buf.len() as u16 <= capacity(self.kind));

        let t_cs = min_tcs_ns(self.kind);
        let mut timer = self.timer.borrow_mut();

        // Enable write.
        chip.select();
        spi.write(&[Opcode::WREN.val()]).await;
        chip.deselect();

        // Wait until we can send a new spi command.
        timer.sleep_ns(t_cs).await;

        // See if write was enabled (it may have been disabled by the WP pin).
        let sr = self.read_status_register(spi, chip).await;

        if sr.wel() {
            let mut write_enabled = true;
            for (address, slice) in buf.aligned_chunks(origin as usize, PAGE_SIZE) {
                if !write_enabled {
                    // Enable write.
                    chip.select();
                    spi.write(&[Opcode::WREN.val()]).await;
                    chip.deselect();
                }

                // Wait until we can send a new spi command.
                timer.sleep_ns(t_cs).await;

                self.write_page(spi, chip, &mut timer, address as u16, slice)
                    .await;

                // Write is auto-disabled after sending a WRITE command.
                write_enabled = false;
            }
            Ok(())
        } else {
            Err(WriteProtectError)
        }
    }

    async fn write_page<Spi, Chip>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        timer: &mut Timer,
        address: u16,
        buf: &[u8],
    ) where
        Spi: At250x0Spi<A>,
        Chip: At250x0Chip<A>,
    {
        assert!(buf.len() > 0);
        assert!(buf.len() <= PAGE_SIZE - (address as usize % PAGE_SIZE));

        chip.select();
        spi.write(&[Opcode::WRITE(address).val(), (address & 0xFF) as u8])
            .await;
        spi.write(buf).await;
        chip.deselect();

        // Wait for idle.
        timer.sleep_us(INITIAL_TIMEOUT_US).await;
        let sr = self.read_status_register(spi, chip).await;
        if sr.bsy() {
            loop {
                timer.sleep_us(RETRY_INTERVAL_US).await;

                let sr = self.read_status_register(spi, chip).await;
                if !sr.bsy() {
                    break;
                }
            }
        }
    }

    async fn read_status_register<Spi, Chip>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
    ) -> StatusRegister
    where
        Spi: At250x0Spi<A>,
        Chip: At250x0Chip<A>,
    {
        const CMD: [u8; 2] = [Opcode::RDSR.val(), 0x00];
        let mut rx: [u8; 2] = [0x00, 0x00];

        chip.select();
        spi.xfer(&CMD, &mut rx).await;
        chip.deselect();

        StatusRegister(rx[1])
    }
}

/// Get the EEPROM capacity in bytes
const fn capacity(kind: At250x0Kind) -> u16 {
    match kind {
        At250x0Kind::At25010 => 128,
        At250x0Kind::At25020 => 256,
        At250x0Kind::At25040 => 512,
        At250x0Kind::At25010b => 128,
        At250x0Kind::At25020b => 256,
        At250x0Kind::At25040b => 512,
    }
}

/// Get the minimum t_cs time in ns, i.e. the minimum time the CS pin must be de-asserted betweeen commands.
const fn min_tcs_ns(kind: At250x0Kind) -> u32 {
    match kind {
        At250x0Kind::At25010 => 250,
        At250x0Kind::At25020 => 250,
        At250x0Kind::At25040 => 250,
        At250x0Kind::At25010b => 100,
        At250x0Kind::At25020b => 100,
        At250x0Kind::At25040b => 100,
    }
}
