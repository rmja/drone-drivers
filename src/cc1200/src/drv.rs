use crate::{
    opcode::{ExtReg, Opcode, Reg, Strobe},
    Cc1200Chip, Cc1200Port, Cc1200Spi,
};
use alloc::sync::Arc;
use core::cell::Cell;
use core::marker::PhantomData;
use drone_core::bitfield::Bitfield;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::future::{self, Either};

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved(r, 0, 4),
    state(r, 4, 3, "Indicates the current main state machine mode."),
    chip_rdy(r, 7, 1, "Stays high until power and crystal have stabilized. Should always be low when using the SPI interface."),
)]
struct StatusRegister(u8);

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[repr(u8)]
enum Cc1200State {
    IDLE = 0b000,
    RX = 0b001,
    TX = 0b010,
    FSTXON = 0b011,
    CALIBRATE = 0b100,
    SETTLING = 0b101,
    RX_FIFO_ERROR = 0b110,
    TX_FIFO_ERROR = 0b111,
}

pub struct Cc1200Drv<Port: Cc1200Port, Al: Alarm<T>, T: Tick, A> {
    port: Port,
    alarm: Arc<Al>,
    adapters: PhantomData<A>,
    status: Cell<StatusRegister>,
    rssi_offset: Rssi,
    tick: PhantomData<T>,
}

const RX_FIFO_SIZE: usize = 128;

pub struct Rssi(i8);

#[derive(Debug)]
pub struct TimeoutError;

impl<Port: Cc1200Port, Al: Alarm<T>, T: Tick, A> Cc1200Drv<Port, Al, T, A> {
    pub fn init(mut port: Port, alarm: Arc<Al>) -> Self {
        port.set_reset(); // Release chip reset pin.

        Self {
            port,
            alarm,
            adapters: PhantomData,
            status: Cell::new(StatusRegister(0)),
            rssi_offset: Rssi(0),
            tick: PhantomData,
        }
    }

    pub async fn hw_reset<Chip: Cc1200Chip<A>>(
        &mut self,
        chip: &mut Chip,
    ) -> Result<(), TimeoutError> {
        // Reset chip.
        self.port.clear_reset(); // Trigger chip reset pin.
        self.alarm.sleep(TimeSpan::from_millis(2)).await;
        self.port.set_reset(); // Release chip reset pin.

        // Wait for chip to become available.
        chip.select();
        self.alarm.sleep(TimeSpan::from_millis(1)).await; // Wait 1ms until the chip has had a chance to set the SO pin high.
        let result = self.wait_for_xtal().await;
        chip.deselect();

        result
    }

    pub async fn read_part_number<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
    ) -> u8 {
        const CMD_LEN: usize = 3;
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::PARTNUMBER as u8,
            0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        chip.select();
        spi.xfer(&CMD, &mut rx_buf).await;
        chip.deselect();
        self.status.set(StatusRegister(rx_buf[0]));

        rx_buf[2]
    }

    /// Write values to the config register section.
    pub fn write_config() {}

    /// Write values to the extended config register section.
    pub fn write_ext_config() {}

    pub async fn read_rssi<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
    ) -> Rssi {
        const CMD_LEN: usize = 3;
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::RSSI1 as u8,
            0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        chip.select();
        spi.xfer(&CMD, &mut rx_buf).await;
        self.status.set(StatusRegister(rx_buf[0]));
        chip.deselect();

        Rssi(rx_buf[2] as i8 + self.rssi_offset.0)
    }

    pub async fn read_fifo<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        buf: &mut [u8],
    ) {
        assert!(buf.len() <= RX_FIFO_SIZE);
        const CMD_LEN: usize = 1 + RX_FIFO_SIZE;
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadFifoBurst.val(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        chip.select();
        spi.xfer(&CMD, &mut rx_buf).await;
        self.status.set(StatusRegister(rx_buf[0]));
        chip.deselect();

        let len = buf.len();
        buf[..len].copy_from_slice(&rx_buf[1..(1 + len)]);
    }

    pub async fn read_rssi_fifo<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        buf: &mut [u8],
    ) -> Rssi {
        const CMD_LEN: usize = 3 + 1 + RX_FIFO_SIZE;
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::RSSI1 as u8,
            0,
            Opcode::ReadFifoBurst.val(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        chip.select();
        spi.xfer(&CMD, &mut rx_buf).await;
        self.status.set(StatusRegister(rx_buf[0]));
        chip.deselect();

        let len = buf.len();
        buf[..len].copy_from_slice(&rx_buf[1..(1 + len)]);
        Rssi(rx_buf[2] as i8 + self.rssi_offset.0)
    }

    /// Wait for the xtal to stabilize.
    async fn wait_for_xtal(&mut self) -> Result<(), TimeoutError> {
        let rising = self.port.miso_wait_low();
        let timeout = self.alarm.sleep(TimeSpan::from_secs(2));

        // Wait for any of the two futures to complete.
        match future::select(rising, timeout).await {
            Either::Left(_) => Ok(()),
            Either::Right(_) => Err(TimeoutError),
        }
    }

    async fn strobe<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &mut self,
        spi: &mut Spi,
        chip: &mut Chip,
        strobe: Strobe,
    ) {
        let tx_buf: [u8; 1] = [Opcode::Strobe(strobe).val()];
        let mut rx_buf = [0u8];

        chip.select();
        spi.xfer(&tx_buf, &mut rx_buf).await;
        self.status.set(StatusRegister(rx_buf[0]));

        if strobe == Strobe::SRES {
            // When SRES strobe is issued the CSn pin must be kept low until the SO pin goes low again.
            self.port.miso_wait_low().await;
        }

        chip.deselect();
    }

    async fn strobe_until<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Pred>(
        &mut self,
        spi: &mut Spi,
        chip: &mut Chip,
        strobe: Strobe,
        pred: Pred,
    ) where
        Pred: Fn(StatusRegister) -> bool,
    {
        let tx_buf: [u8; 1] = [strobe as u8];
        let mut rx_buf = [0u8];
        assert_ne!(Strobe::SRES, strobe);

        chip.select();
        loop {
            spi.xfer(&tx_buf, &mut rx_buf).await;
            self.status.set(StatusRegister(rx_buf[0]));

            if pred(self.status.get()) {
                break;
            }
        }
        chip.deselect();
    }
}
