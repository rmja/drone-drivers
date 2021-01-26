use drone_cc1200_drv::Cc1200Uptime;
use drone_stm32f4_hal::rcc::clktree::*;
use drone_time::{Tick, TimeSpan, Uptime, UptimeCounter, UptimeDrv, UptimeOverflow};

pub const HSECLK: HseClk = HseClk::new(8_000_000);
pub const PLLSRC_HSECLK: PllSrcMuxSignal = PllSrcMuxSignal::Hse(HSECLK);
pub const PLL: Pll = PLLSRC_HSECLK.to_pllsrc(8).to_pll(360, 2, 8);
pub const SYSCLK_PLL: SysClkMuxSignal = SysClkMuxSignal::Pll(PLL.p);
pub const SYSCLK: SysClk = SYSCLK_PLL.to_sysclk();
pub const HCLK: HClk = SYSCLK.to_hclk(1);
pub const PCLK1: PClk1 = HCLK.to_pclk1(4);
pub const PCLK2: PClk2 = HCLK.to_pclk2(2);
pub const SYSTICKCLK: SysTickClk = HCLK.to_systickclk();

pub const TIM2_FREQ: u32 = 1_000_000;

pub struct Tim2Tick;
impl Tick for Tim2Tick {
    const FREQ: u32 = TIM2_FREQ;
}

impl<Cnt: UptimeCounter<Tim2Tick, A>, Ovf: UptimeOverflow<A>, A: Send + Sync + 'static>
    Cc1200Uptime<Tim2Tick, drone_cc1200_drv::drivers::stm32f4::Adapter>
    for UptimeDrv<Tim2Tick, Cnt, Ovf, A>
{
    fn upstamp(&self, capture: u32) -> TimeSpan<Tim2Tick> {
        // The uptime counter and the captured value are synchronous.
        self.at(capture)
    }
}
