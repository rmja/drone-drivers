pub(crate) mod cc1200 {
    use async_trait::async_trait;
    use drone_cc1200_drv::Cc1200Port;
    use drone_cortexm::thr::prelude::*;
    use drone_stm32_map::periph::{
        exti::{ExtiFtsrFt, ExtiMap, ExtiPrPif, ExtiRtsrRt, ExtiSwierSwi, SyscfgExticrExti},
        gpio::pin::GpioPinMap,
    };
    use drone_stm32f4_hal::{
        exti::{prelude::*, ExtiLine},
        gpio::{prelude::*, GpioPin},
    };

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
        pub miso_exti_line: ExtiLine<
            'a,
            MisoExti,
            MisoExtiInt,
            MisoPin,
            AlternateMode<MisoAf>,
            PushPullType,
            PullDown, // The miso pin should be pull'ed accourding to the cc1200 IOCFG1 description.
            FallingEdge,
        >,
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
}
