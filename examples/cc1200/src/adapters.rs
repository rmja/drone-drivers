pub struct Adapter;

pub(crate) mod cc1200 {
    use async_trait::async_trait;
    use drone_cc1200_drv::Cc1200Port;
    use drone_cortexm::thr::prelude::*;
    use drone_stm32f4_hal::{
        exti::{prelude::*, ExtiLine, ExtiMap},
        gpio::{prelude::*, GpioPin, GpioPinMap},
    };

    pub struct Port<
        ResetPin: GpioPinMap,
        MisoPin: GpioPinMap,
        MisoAf: PinAf,
        MisoExti: ExtiMap,
        MisoExtiInt: IntToken,
    > {
        pub reset_pin: GpioPin<ResetPin, OutputMode, PushPullType, PullUp>,
        pub miso_exti_line: ExtiLine<
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
            MisoExti: ExtiMap,
            MisoExtiInt: IntToken,
        > Cc1200Port for Port<ResetPin, MisoPin, MisoAf, MisoExti, MisoExtiInt>
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
