mod at250x0b {
    use async_trait::async_trait;
    use drone_at250x0_drv::{At250x0Chip, At250x0Spi, At250x0Timer};
    use drone_core::thr::ThrToken;
    use drone_cortexm::{
        drv::{sys_tick::SysTick, timer::Timer},
        thr::IntToken,
    };
    use drone_stm32_map::periph::{
        dma::ch::DmaChMap,
        gpio::pin::GpioPinMap,
        spi::{SpiCr1, SpiMap},
    };
    use drone_stm32f4_hal::{
        gpio::prelude::*,
        spi::{chipctrl::SpiChip, SpiMasterDrv},
    };

    pub struct Adapters;

    #[async_trait]
    impl<Int: ThrToken> At250x0Timer<Adapters> for SysTick<Int> {
        async fn sleep_ns(&mut self, duration: u32) {
            self.sleep((duration * (crate::consts::SYSTICKCLK.f() / 1000)) / 1_000_000);
        }

        async fn sleep_us(&mut self, duration: u32) {
            self.sleep((duration * (crate::consts::SYSTICKCLK.f() / 1000)) / 1000);
        }
    }

    impl<Pin: GpioPinMap, PinType: PinTypeToken, PinPull: PinPullToken> At250x0Chip<Adapters>
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
            Spi: SpiMap + SpiCr1,
            DmaRx: DmaChMap,
            DmaRxInt: IntToken,
            DmaTx: DmaChMap,
            DmaTxInt: IntToken,
        > At250x0Spi<Adapters> for SpiMasterDrv<'_, Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>
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
}
