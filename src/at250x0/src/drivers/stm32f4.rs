use crate::{At250x0Chip, At250x0Spi};
use drone_stm32f4_hal::{IntToken, spi::{SpiMap, SpiMasterDrv, chipctrl::SpiChip}, gpio::GpioPinMap, dma::DmaChMap};
use async_trait::async_trait;

pub struct Adapter;

impl<Pin: GpioPinMap, PinType, PinPull> At250x0Chip<Adapter>
        for SpiChip<Pin, PinType, PinPull>
    {
        fn select(&mut self) {
            self.select();
        }

        fn deselect(&mut self) {
            self.deselect();
        }
    }

    #[async_trait]
    impl<
            Spi: SpiMap,
            DmaRx: DmaChMap,
            DmaRxInt: IntToken,
            DmaTx: DmaChMap,
            DmaTxInt: IntToken,
        > At250x0Spi<Adapter> for SpiMasterDrv<'_, Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>
    {
        async fn read(&mut self, rx: &mut [u8]) {
            self.read(rx).await;
        }

        async fn write(&mut self, tx: &[u8]) {
            self.write(tx).await;
        }

        async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]) {
            self.xfer(tx, rx).await;
        }
    }