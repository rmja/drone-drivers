use crate::{
    opcode::{ExtReg, Opcode, Reg, Strobe},
    Cc1200Chip, Cc1200Config, Cc1200PartNumber, Cc1200Port, Cc1200Spi, Rssi, State, StatusByte,
};
use alloc::sync::Arc;
use drone_core::sync::Mutex;
use core::cell::{Cell, RefCell};
use core::marker::PhantomData;
use drone_time::{Alarm, Tick, TimeSpan};
use futures::future::{self, Either};

pub struct Cc1200Drv<Port: Cc1200Port, Al: Alarm<T>, T: Tick, A> {
    port: RefCell<Port>,
    pub alarm: Arc<Al>,
    status: Cell<StatusByte>,
    rssi_offset: Rssi,
    tick: PhantomData<T>,
    adapter: PhantomData<A>,
}

pub const RX_FIFO_SIZE: usize = 128;
pub const TX_FIFO_SIZE: usize = 128;

#[derive(Debug)]
pub struct TimeoutError;

#[derive(Debug)]
pub struct InvalidPartNumber;

#[derive(Debug)]
pub struct InvalidRssi;

impl<Port: Cc1200Port, Al: Alarm<T>, T: Tick, A> Cc1200Drv<Port, Al, T, A> {
    pub fn init(mut port: Port, alarm: Arc<Al>) -> Self {
        port.set_reset(); // Release chip reset pin.

        Self {
            port: RefCell::new(port),
            alarm,
            status: Cell::new(StatusByte(0)),
            rssi_offset: Rssi(0),
            tick: PhantomData,
            adapter: PhantomData,
        }
    }

    pub async fn hw_reset<Chip: Cc1200Chip<A>>(&self, chip: &mut Chip) -> Result<(), TimeoutError> {
        // Reset chip.
        let mut port = self.port.borrow_mut();
        port.clear_reset(); // Trigger chip reset pin.
        self.alarm.sleep(TimeSpan::from_millis(2)).await;
        port.set_reset(); // Release chip reset pin.

        // Wait for chip to become available.
        chip.select();
        self.alarm.sleep(TimeSpan::from_millis(1)).await; // Wait 1ms until the chip has had a chance to set the SO pin high.
        let result = self.wait_for_xtal(&mut port).await;
        chip.deselect();

        result
    }

    /// Get the spi status returned by the last register read or strobe.
    /// Writing registers does not update status.
    pub fn last_status(&self) -> StatusByte {
        self.status.get()
    }

    /// Read the chip part number.
    /// This action _does_ update `last_status`.
    pub async fn read_part_number<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
    ) -> Result<Cc1200PartNumber, InvalidPartNumber> {
        let mut buf = [0];
        self.read_ext_regs(spi, chip, ExtReg::PARTNUMBER, &mut buf)
            .await;
        match buf[0] {
            0x20 => Ok(Cc1200PartNumber::Cc1200),
            0x21 => Ok(Cc1200PartNumber::Cc1201),
            _ => Err(InvalidPartNumber),
        }
    }

    /// Write configuration values to chip.
    /// This action _does not_ update `last_status`.
    pub async fn write_config<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        config: &Cc1200Config<'_>,
    ) {
        if !config.values.is_empty() {
            self.write_regs(spi, chip, config.first, config.values)
                .await;
        }

        if !config.ext_values.is_empty() {
            self.write_ext_regs(spi, chip, config.ext_first, config.ext_values)
                .await;
        }
    }

    /// Read a sequence of register values from chip.
    /// This action _does_ update `last_status`.
    pub async fn read_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: Reg,
        buf: &mut [u8],
    ) {
        let len = buf.len();
        let opcode = if len == 1 {
            Opcode::ReadSingle(first)
        } else {
            Opcode::ReadBurst(first)
        };
        let tx = &[opcode.val()];
        let mut rx_buf = [0];

        chip.select();
        spi.xfer(tx, &mut rx_buf).await;
        self.status.set(StatusByte(rx_buf[0]));
        spi.read(buf).await;
        chip.deselect();
    }

    /// Read a sequence of extended register values from chip.
    /// This action _does_ update `last_status`.
    pub async fn read_ext_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: ExtReg,
        buf: &mut [u8],
    ) {
        let len = buf.len();
        let opcode = if len == 1 {
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS)
        } else {
            Opcode::ReadBurst(Reg::EXTENDED_ADDRESS)
        };
        let tx = &[opcode.val(), first as u8];
        let mut rx_buf = [0, 0];

        chip.select();
        spi.xfer(tx, &mut rx_buf).await;
        self.status.set(StatusByte(rx_buf[0]));
        spi.read(buf).await;
        chip.deselect();
    }

    /// Write register values.
    /// This action _does not_ update `last_status`.
    pub async fn write_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: Reg,
        values: &[u8],
    ) {
        let len = values.len();
        let opcode = if len == 1 {
            Opcode::WriteSingle(first)
        } else {
            Opcode::WriteBurst(first)
        };
        let mut tx = Vec::with_capacity(1 + len);
        tx.push(opcode.val());
        tx.extend_from_slice(values);

        chip.select();
        spi.write(&tx).await;
        chip.deselect();
    }

    /// Write extended register values.
    /// This action _does not_ update `last_status`.
    pub async fn write_ext_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: ExtReg,
        values: &[u8],
    ) {
        let len = values.len();
        let opcode = if len == 1 {
            Opcode::WriteSingle(Reg::EXTENDED_ADDRESS)
        } else {
            Opcode::WriteBurst(Reg::EXTENDED_ADDRESS)
        };
        let mut tx = Vec::with_capacity(2 + len);
        tx.push(opcode.val());
        tx.push(first as u8);
        tx.extend_from_slice(values);

        chip.select();
        spi.write(&tx).await;
        chip.deselect();
    }

    /// Modify register values.
    /// This action _does_ update `last_status`.
    pub async fn modify_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, F: FnOnce(&mut [u8])>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: Reg,
        count: usize,
        configure: F,
    ) {
        let mut buf = vec![0; count];
        self.read_regs(spi, chip, first, &mut buf).await;
        configure(&mut buf);
        self.write_regs(spi, chip, first, &buf).await;
    }

    /// Modify extended register values.
    /// This action _does_ update `last_status`.
    pub async fn modify_ext_regs<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, F: FnOnce(&mut [u8])>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        first: ExtReg,
        count: usize,
        configure: F,
    ) {
        let mut buf = vec![0; count];
        self.read_ext_regs(spi, chip, first, &mut buf).await;
        configure(&mut buf);
        self.write_ext_regs(spi, chip, first, &buf).await;
    }

    /// Read the current RSSI level.
    /// This action _does_ update `last_status`.
    pub async fn read_rssi<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
    ) -> Result<Rssi, InvalidRssi> {
        const CMD_LEN: usize = 3;
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::RSSI1 as u8,
            0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        chip.select();
        spi.xfer(&CMD, &mut rx_buf).await;
        self.status.set(StatusByte(rx_buf[0]));
        chip.deselect();

        let rssi = rx_buf[2] as i8;
        match rssi {
            -128 => Err(InvalidRssi),
            rssi => Ok(Rssi(rssi + self.rssi_offset.0)),
        }
    }

    /// Read from the RX fifo.
    /// This action _does_ update `last_status`.
    pub async fn read_fifo<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        buf: &mut [u8],
    ) {
        assert!(buf.len() <= RX_FIFO_SIZE);
        const CMD_LEN: usize = 1 + RX_FIFO_SIZE;
        #[rustfmt::skip]
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadFifoBurst.val(),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        let len = 1 + buf.len();
        chip.select();
        spi.xfer(&CMD[..len], &mut rx_buf[..len]).await;
        self.status.set(StatusByte(rx_buf[0]));
        chip.deselect();

        buf.copy_from_slice(&rx_buf[1..len]);
    }

    /// Read the RSSI and RX fifo in one transaction.
    /// This action _does_ update `last_status`.
    pub async fn read_rssi_and_fifo<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        buf: &mut [u8],
    ) -> Rssi {
        const CMD_LEN: usize = 3 + 1 + RX_FIFO_SIZE;
        #[rustfmt::skip]
        const CMD: [u8; CMD_LEN] = [
            Opcode::ReadSingle(Reg::EXTENDED_ADDRESS).val(),
            ExtReg::RSSI1 as u8,
            0,
            Opcode::ReadFifoBurst.val(),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ];
        let mut rx_buf: [u8; CMD_LEN] = [0; CMD_LEN];

        let len = 3 + 1 + buf.len();
        chip.select();
        spi.xfer(&CMD[..len], &mut rx_buf[..len]).await;
        self.status.set(StatusByte(rx_buf[0]));
        chip.deselect();

        buf.copy_from_slice(&rx_buf[4..len]);

        let rssi = rx_buf[2] as i8;
        assert_ne!(-128, rssi);
        Rssi(rssi + self.rssi_offset.0)
    }

    /// Write to the TX fifo.
    /// This action _does_ update `last_status`.
    pub async fn write_fifo<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        buf: &[u8],
    ) {
        let mut rx_buf: [u8; 1] = [0; 1];

        chip.select();
        spi.xfer(&[Opcode::WriteFifoBurst.val()], &mut rx_buf).await;
        self.status.set(StatusByte(rx_buf[0]));
        spi.write(buf).await;
        chip.deselect();
    }

    /// Wait for the xtal to stabilize.
    async fn wait_for_xtal(&self, port: &mut Port) -> Result<(), TimeoutError> {
        let rising = port.miso_wait_low();
        let timeout = self.alarm.sleep(TimeSpan::from_secs(2));

        // Wait for any of the two futures to complete.
        match future::select(rising, timeout).await {
            Either::Left(_) => Ok(()),
            Either::Right(_) => Err(TimeoutError),
        }
    }

    /// Strobe a command to the chip.
    /// This action _does_ update `last_status`.
    pub async fn strobe<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        strobe: Strobe,
    ) {
        let tx_buf: [u8; 1] = [Opcode::Strobe(strobe).val()];
        let mut rx_buf = [0u8];

        chip.select();
        spi.xfer(&tx_buf, &mut rx_buf).await;
        self.status.set(StatusByte(rx_buf[0]));

        if strobe == Strobe::SRES {
            // When SRES strobe is issued the CSn pin must be kept low until the SO pin goes low again.
            let mut port = self.port.borrow_mut();
            port.miso_wait_low().await;
        }

        chip.deselect();
    }

    /// Strobe a command to the chip, and continue to do so until `pred` is satisfied.
    /// This action _does_ update `last_status`.
    pub async fn strobe_until<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>, Pred>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        strobe: Strobe,
        pred: Pred,
    ) where
        Pred: Fn(StatusByte) -> bool,
    {
        let tx_buf: [u8; 1] = [strobe as u8];
        let mut rx_buf = [0u8];
        assert_ne!(Strobe::SRES, strobe);

        chip.select();
        loop {
            spi.xfer(&tx_buf, &mut rx_buf).await;
            self.status.set(StatusByte(rx_buf[0]));

            if pred(self.status.get()) {
                break;
            }
        }
        chip.deselect();
    }

    /// Strobe a command to the chip, and continue to do so until the chip enters the IDLE state.
    /// This action _does_ update `last_status`.
    pub async fn strobe_until_idle<Spi: Cc1200Spi<A>, Chip: Cc1200Chip<A>>(
        &self,
        spi: &mut Spi,
        chip: &mut Chip,
        strobe: Strobe,
    ) {
        self.strobe_until(spi, chip, strobe, |status| status.state() == State::IDLE)
            .await;
    }
}
