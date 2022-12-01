/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. De-asserted
/// when the RX FIFO is drained below (or is equal) to the same threshold.
const GPIOCFG_RXFIFO_THR: u8 = 0;
/// Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR or the end of
/// packet is reached. De-asserted when the RX FIFO is empty.
const GPIOCFG_RXFIFO_THR_PKT: u8 = 1;
/// Asserted when the TX FIFO is filled above (or is equal to)
/// (127−FIFO_CFG.FIFO_THR). De-asserted when the TX FIFO is drained below the
/// same threshold.
const GPIOCFG_TXFIFO_THR: u8 = 2;
/// Asserted when the TX FIFO is full.
/// De-asserted when the TX FIFO is drained below (127−FIFO_CFG.FIFO_THR).
const GPIOCFG_TXFIFO_THR_PKT: u8 = 3;
const GPIOCFG_RXFIFO_OVERFLOW: u8 = 4;
const GPIOCFG_TXFIFO_UNDERFLOW: u8 = 5;
/// Asserted when sync word has been received and de-asserted at the end of the
/// packet. Will de-assert when the optional address and/or length check fails
/// or the RX FIFO overflows/underflows
const GPIOCFG_PKT_SYNC_RXTX: u8 = 6;
const GPIOCFG_CRC_OK: u8 = 7;
const GPIOCFG_SERIAL_CLK: u8 = 8;
const GPIOCFG_SERIAL_RX: u8 = 9;
const GPIOCFG_PQT_REACHED: u8 = 11;
const GPIOCFG_PQT_VALID: u8 = 12;
/// RSSI calculation is valid
const GPIOCFG_RSSI_VALID: u8 = 13;
const GPIOCFG_CARRIER_SENSE_VALID: u8 = 16;
const GPIOCFG_CARRIER_SENSE: u8 = 17;
const GPIOCFG_PKT_CRC_OK: u8 = 19;
const GPIOCFG_MCU_WAKEUP: u8 = 20;
const GPIOCFG_SYNC_LOW0_HIGH1: u8 = 21;
const GPIOCFG_LNA_PA_REG_PD: u8 = 23;
const GPIOCFG_LNA_PD: u8 = 24;
const GPIOCFG_PA_PD: u8 = 25;
const GPIOCFG_RX0TX1_CFG: u8 = 26;
const GPIOCFG_IMAGE_FOUND: u8 = 28;
const GPIOCFG_CLKEN_CFM: u8 = 29;
const GPIOCFG_CFM_TX_DATA_CLK: u8 = 30;
const GPIOCFG_RSSI_STEP_FOUND: u8 = 33;
const GPIOCFG_ANTENNA_SELECT: u8 = 36;
const GPIOCFG_MARC_2PIN_STATUS1: u8 = 37;
const GPIOCFG_MARC_2PIN_STATUS0: u8 = 38;
const GPIOCFG_PA_RAMP_UP: u8 = 42;
const GPIOCFG_AGC_STABLE_GAIN: u8 = 44;
const GPIOCFG_AGC_UPDATE: u8 = 45;
const GPIOCFG_HIGHZ: u8 = 48;
const GPIOCFG_EXT_CLOCK: u8 = 49;
const GPIOCFG_CHIP_RDYn: u8 = 50;
const GPIOCFG_HW0: u8 = 51;
const GPIOCFG_CLOCK_40K: u8 = 54;
const GPIOCFG_WOR_EVENT0: u8 = 55;
const GPIOCFG_WOR_EVENT1: u8 = 56;
const GPIOCFG_WOR_EVENT2: u8 = 57;
const GPIOCFG_XOSC_STABLE: u8 = 59;
const GPIOCFG_EXT_OSC_EN: u8 = 60;
