use drone_time::Tick;
use crate::consts;

pub struct Tim2Tick;
impl Tick for Tim2Tick {
    const FREQ: u32 = consts::TIM2_FREQ;
}

pub(crate) mod cc1200 {
    use async_trait::async_trait;
    use drone_cc1200_drv::{Cc1200Chip, Cc1200Port, Cc1200Spi};
    use drone_cortexm::{
        thr::prelude::*,
    };
    use drone_time::Alarm;
    use drone_stm32_map::periph::{
        dma::ch::DmaChMap,
        exti::{ExtiFtsrFt, ExtiMap, ExtiPrPif, ExtiRtsrRt, ExtiSwierSwi, SyscfgExticrExti},
        gpio::pin::GpioPinMap,
        spi::SpiMap,
        tim::general::GeneralTimMap
    };
    use drone_stm32f4_hal::{exti::{prelude::*, ExtiLine}, gpio::{prelude::*, GpioPin}, spi::{chipctrl::SpiChip, config::MisoPinExt, SpiMasterDrv}, tim::{GeneralTimChDrv, OutputCompareMode}};
    use futures::prelude::*;

    pub struct Adapters;

    pub struct Port<
        'a,
        ResetPin: GpioPinMap,
        MisoPin: GpioPinMap,
        MisoAf: PinAf,
        MisoExti: ExtiMap + SyscfgExticrExti + ExtiRtsrRt + ExtiFtsrFt + ExtiSwierSwi + ExtiPrPif,
        MisoExtiInt: IntToken,
    > {
        pub reset_pin: GpioPin<ResetPin, OutputMode, PushPullType, PullUp>,
        pub miso_exti_line: ExtiLine<'a, MisoExti, MisoExtiInt, MisoPin, AlternateMode<MisoAf>, PushPullType, NoPull, FallingEdge>,
    }

    #[async_trait]
    impl<
            ResetPin: GpioPinMap,
            MisoPin: GpioPinMap,
            MisoAf: PinAf,
            MisoExti: ExtiMap + SyscfgExticrExti + ExtiRtsrRt + ExtiFtsrFt + ExtiSwierSwi + ExtiPrPif,
            MisoExtiInt: IntToken,
        > Cc1200Port for Port<'_, ResetPin, MisoPin, MisoAf, MisoExti, MisoExtiInt>
    {
        fn set_reset(&mut self) {
            self.reset_pin.set();
        }

        fn clear_reset(&mut self) {
            self.reset_pin.clear();
        }

        async fn miso_wait_low(&mut self) {
            self.miso_exti_line.wait_low().await;
        }
    }

    // #[async_trait]
    // impl<Tim: GeneralTimMap, Int: ThrToken, Ch> Cc1200Timer<Adapters> for Alarm<Tim, Int, Ch, OutputCompareMode> {
    //     async fn sleep_ms(&mut self, ms: u32) {
    //         let duration = (ms * crate::consts::SYSTICKCLK.f()) / 1000;
    //         self.sleep(duration).await;
    //     }
    // }

    impl<Pin: GpioPinMap, PinType, PinPull> Cc1200Chip<Adapters>
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
        > Cc1200Spi<Adapters> for SpiMasterDrv<'_, Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>
    {
        async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]) {
            self.xfer(tx, rx).await;
        }
    }
}
