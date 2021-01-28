//! The root task.

use crate::{consts, thr, thr::ThrsInit, Regs, tasks};
use alloc::sync::Arc;
use drone_cc1200_drv::{
    configs::CC1200_WMBUS_MODECMTO_FULL,
    controllers::{DebugController, InfiniteController},
    Cc1200Drv, Cc1200Gpio, Cc1200PartNumber,
};
use drone_core::{log, sync::Mutex};
use drone_cortexm::{
    drv::sys_tick::SysTick, periph_sys_tick, reg::prelude::*, swo, thr::prelude::*,
};
use drone_stm32_map::periph::{
    dma::{periph_dma2, periph_dma2_ch2, periph_dma2_ch3},
    exti::periph_exti6,
    gpio::{
        periph_gpio_a5, periph_gpio_a6, periph_gpio_a7, periph_gpio_a_head, periph_gpio_b0,
        periph_gpio_b8, periph_gpio_b_head, periph_gpio_d12, periph_gpio_d_head, periph_gpio_i1,
        periph_gpio_i_head,
    },
    spi::periph_spi1,
    tim::{periph_tim2, periph_tim4},
};
use drone_stm32f4_hal::{
    dma::{config::*, DmaCfg},
    exti::{periph_syscfg, prelude::*, ExtiDrv, Syscfg},
    gpio::{prelude::*, GpioHead},
    rcc::{prelude::*, periph_flash, periph_pwr, periph_rcc, Flash, Pwr, Rcc, RccSetup},
    spi::{chipctrl::*, prelude::*, SpiDrv, SpiPins, SpiSetup},
    tim::{prelude::*, GeneralTimCfg, GeneralTimSetup},
};
use drone_time::{AlarmDrv, UptimeDrv};

/// The root task handler.
#[inline(never)]
pub fn handler(reg: Regs, thr_init: ThrsInit) {
    handle(reg, thr_init).root_wait();
}

async fn handle(reg: Regs, thr_init: ThrsInit) {
    let thr = thr::init(thr_init);

    thr.hard_fault.add_once(|| panic!("Hard Fault"));

    println!("Hello, world!");

    // Enable interrupts.
    thr.rcc.enable_int();
    thr.exti_9_5.enable_int();
    thr.tim_2.enable_int();
    thr.tim_4.enable_int();
    thr.spi_1.enable_int();
    thr.dma_2_ch_2.enable_int();
    thr.dma_2_ch_3.enable_int();

    thr.rf.enable_int();

    // Initialize clocks.
    let rcc = Rcc::init(RccSetup::new(periph_rcc!(reg), thr.rcc));
    let pwr = Pwr::with_enabled_clock(periph_pwr!(reg));
    let flash = Flash::new(periph_flash!(reg));

    let hseclk = rcc.stabilize(consts::HSECLK).await;
    let pll = rcc
        .select(consts::PLLSRC_HSECLK, hseclk)
        .stabilize(consts::PLL)
        .await;
    let hclk = rcc.configure(consts::HCLK);
    let pclk1 = rcc.configure(consts::PCLK1);
    let pclk2 = rcc.configure(consts::PCLK2);
    pwr.enable_overdrive();
    flash.set_latency(consts::HCLK.get_wait_states(VoltageRange::HighVoltage));
    swo::flush();
    swo::update_prescaler(consts::HCLK.f() / log::baud_rate!() - 1);
    rcc.select(consts::SYSCLK_PLL, pll.p());

    // Enable IO port clock.
    let gpio_a = GpioHead::with_enabled_clock(periph_gpio_a_head!(reg));
    let gpio_b = GpioHead::with_enabled_clock(periph_gpio_b_head!(reg));
    let gpio_d = GpioHead::with_enabled_clock(periph_gpio_d_head!(reg));
    let gpio_i = GpioHead::with_enabled_clock(periph_gpio_i_head!(reg));

    // Configure SPI GPIO pins.
    let sck_pin = gpio_a
        .pin(periph_gpio_a5!(reg))
        .into_alternate()
        .into_pushpull()
        .into_nopull()
        .with_speed(GpioPinSpeed::MediumSpeed);
    let miso_pin = gpio_a
        .pin(periph_gpio_a6!(reg))
        .into_alternate()
        .into_pushpull()
        .into_pulldown()
        .with_speed(GpioPinSpeed::MediumSpeed);
    let mosi_pin = gpio_a
        .pin(periph_gpio_a7!(reg))
        .into_alternate()
        .into_pushpull()
        .into_nopull()
        .with_speed(GpioPinSpeed::MediumSpeed);
    let cs_pin = gpio_b
        .pin(periph_gpio_b0!(reg))
        .into_output()
        .into_pushpull()
        .into_pullup()
        .with_speed(GpioPinSpeed::MediumSpeed);

    let miso_pin_reset = unsafe { miso_pin.clone() };

    // Initialize exti.
    let syscfg = Syscfg::with_enabled_clock(periph_syscfg!(reg));
    let exti6 = Arc::new(ExtiDrv::new(periph_exti6!(reg), thr.exti_9_5, &syscfg).into_falling_edge());

    // Initialize dma.
    let dma2 = DmaCfg::with_enabled_clock(periph_dma2!(reg));
    let miso_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch2!(reg), thr.dma_2_ch_2));
    let mosi_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch3!(reg), thr.dma_2_ch_3));

    // Initialize spi.
    let pins = SpiPins::default()
        .sck(sck_pin)
        .miso(miso_pin)
        .mosi(mosi_pin);
    let setup = SpiSetup::new(
        periph_spi1!(reg),
        thr.spi_1,
        pins,
        pclk2,
        BaudRate::Max(7_700_000),
    );
    let mut spi = SpiDrv::init(setup).into_master(miso_dma, mosi_dma);

    let mut chip = SpiChip::new_deselected(cs_pin);

    // Deselect other SPI devices on same bus
    SpiChip::new_deselected(
        gpio_i
            .pin(periph_gpio_i1!(reg))
            .into_output()
            .into_pushpull(),
    );

    // Initialize CC1200 gpio pins
    let dr_pin = gpio_d
        .pin(periph_gpio_d12!(reg))
        .into_alternate()
        .into_pushpull()
        .into_pulldown()
        .with_speed(GpioPinSpeed::MediumSpeed);
    let reset_pin = gpio_b
        .pin(periph_gpio_b8!(reg))
        .into_output()
        .into_pushpull()
        .into_pullup();

    // Initialize timer.
    let tim2 = GeneralTimCfg::with_enabled_clock(GeneralTimSetup::new(
        periph_tim2!(reg),
        thr.tim_2,
        pclk1,
        TimFreq::Nominal(consts::TIM2_FREQ),
    ))
    .into_count_up()
    .ch1(|ch| ch.into_output_compare())
    .into_master();

    let mut tim4 = GeneralTimCfg::with_enabled_clock(GeneralTimSetup::new(
        periph_tim4!(reg),
        thr.tim_4,
        pclk1,
        TimFreq::Nominal(consts::TIM2_FREQ),
    ))
    .into_count_up()
    .ch1(|ch| ch.into_input_capture_pin(dr_pin))
    .into_trigger_slave_of(tim2.link);

    tim2.start(); // Also starts tim4

    let uptime = UptimeDrv::new(
        tim2.counter.clone(),
        tim2.overflow,
        thr.tim_2,
        consts::Tim2Tick,
    );
    let alarm = Arc::new(AlarmDrv::new(tim2.counter, tim2.ch1, consts::Tim2Tick));

    let spi = Arc::new(Mutex::new(spi));

    // Initialize CC1200
    // LA Colors:
    // 0. MISO: Grey
    // 2. RF_CS: Red
    // 3. EE_CS: Orange
    // 4. CLK: Yellow
    // 5. DATA_RDY: Green
    // 6. MOSI: Blue
    // 7. RESET: Purple
    let port = crate::adapters::cc1200::Port {
        reset_pin,
        miso_exti_line: exti6.line(miso_pin_reset),
    };
    thr.rf.exec(tasks::rf(port, alarm.clone(), spi.clone(), chip));
    // let mut cc1200 = Cc1200Drv::init(port, alarm.clone());

    // let mut debug = DebugController::setup(cc1200, spi.clone(), chip, &CC1200_WMBUS_MODECMTO_FULL).await.unwrap();

    // debug.tx_unmodulated().await;
    // alarm.sleep(TimeSpan::from_secs(3)).await;
    // debug.tx_modulated_01().await;
    // alarm.sleep(TimeSpan::from_secs(3)).await;
    // debug.tx_modulated_pn9().await;
    // alarm.sleep(TimeSpan::from_secs(3)).await;
    // debug.idle().await;

    // let (cc1200, chip) = debug.release();
    // let mut infinite = InfiniteController::setup(
    //     cc1200,
    //     spi.clone(),
    //     chip,
    //     tim4.ch1,
    //     uptime.clone(),
    //     &CC1200_WMBUS_MODECMTO_FULL,
    //     Cc1200Gpio::Gpio0,
    // )
    // .await
    // .unwrap();

    // let ctrl = Arc::new(Mutex::new(infinite));

    

    // let mut count = 0;
    // let mut rx_stream = infinite.rx_stream(1).await;
    // while let Some(whoot) = receive_stream.next().await {
    //     // println!("{:?}", whoot.upstamp);
    //     count += 1;

    //     if count == 100 {
    //         break;
    //     }
    // }
    // drop(rx_stream);
    // infinite.release();

    // let tx: Vec<u8> = (0..=255).collect();

    // infinite.write(&tx).await;

    // let before = uptime.now();
    // infinite.transmit_packet().await.unwrap();
    // let after = uptime.now();

    // let tx_us = (after - before).as_micros();
    // println!("Tx strobe + actual transmission took {}us", tx_us);

    // Enter a sleep state on ISR exit.
    reg.scb_scr.sleeponexit.set_bit();
}
