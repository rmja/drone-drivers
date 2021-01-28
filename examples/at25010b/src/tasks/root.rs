//! The root task.

use crate::{Regs, consts::{self, SysTickTick}, thr, thr::ThrsInit};
use alloc::sync::Arc;
use drone_at250x0_drv::{At250x0Drv, At250x0Kind};
use drone_core::log;
use drone_cortexm::{reg::prelude::*, swo, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::{periph_dma2, periph_dma2_ch2, periph_dma2_ch3},
    gpio::{
        periph_gpio_a5, periph_gpio_a6, periph_gpio_a7, periph_gpio_a_head, periph_gpio_i1,
        periph_gpio_b_head, periph_gpio_b0,
        periph_gpio_i_head,
    },
    spi::periph_spi1,
    sys_tick::periph_sys_tick,
};
use drone_stm32f4_hal::{
    dma::{config::*, DmaCfg},
    gpio::{prelude::*, GpioHead},
    rcc::{prelude::*, periph_flash, periph_pwr, periph_rcc, Flash, Pwr, Rcc, RccSetup},
    spi::{prelude::*, chipctrl::*, SpiDrv, SpiSetup, SpiPins},
};
use drone_time::{AlarmDrv, drivers::SysTickAlarmDrv};

/// The root task handler.
#[inline(never)]
pub fn handler(reg: Regs, thr_init: ThrsInit) {
    let thr = thr::init(thr_init);

    thr.hard_fault.add_once(|| panic!("Hard Fault"));

    println!("Hello, world!");

    // Enable interrupts.
    thr.rcc.enable_int();
    thr.spi_1.enable_int();
    thr.dma_2_ch_2.enable_int();
    thr.dma_2_ch_3.enable_int();

    // Initialize clocks.
    let rcc = Rcc::init(RccSetup::new(periph_rcc!(reg), thr.rcc));
    let pwr = Pwr::with_enabled_clock(periph_pwr!(reg));
    let flash = Flash::new(periph_flash!(reg));

    let hseclk = rcc.stabilize(consts::HSECLK).root_wait();
    let pll = rcc
        .select(consts::PLLSRC_HSECLK, hseclk)
        .stabilize(consts::PLL)
        .root_wait();
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
    let gpio_i = GpioHead::with_enabled_clock(periph_gpio_i_head!(reg));

    // Configure SPI GPIO pins.
    let pin_sck = gpio_a
        .pin(periph_gpio_a5!(reg))
        .into_alternate()
        .into_pushpull()
        .into_nopull()
        .with_speed(GpioPinSpeed::HighSpeed);
    let pin_miso = gpio_a
        .pin(periph_gpio_a6!(reg))
        .into_alternate()
        .into_pushpull()
        .into_nopull()
        .with_speed(GpioPinSpeed::HighSpeed);
    let pin_mosi = gpio_a
        .pin(periph_gpio_a7!(reg))
        .into_alternate()
        .into_pushpull()
        .into_nopull()
        .with_speed(GpioPinSpeed::HighSpeed);
    let pin_cs = gpio_i
        .pin(periph_gpio_i1!(reg))
        .into_output()
        .into_pushpull()
        .into_pullup()
        .with_speed(GpioPinSpeed::HighSpeed);

    // Deselect other SPI devices on same bus
    gpio_b.pin(periph_gpio_b0!(reg)).into_output().into_pushpull().set();

    // Initialize dma.
    let dma2 = DmaCfg::with_enabled_clock(periph_dma2!(reg));
    let miso_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch2!(reg), thr.dma_2_ch_2));
    let mosi_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch3!(reg), thr.dma_2_ch_3));

    // Initialize spi.
    let pins = SpiPins::default()
        .sck(pin_sck)
        .miso(pin_miso)
        .mosi(pin_mosi);
    let setup = SpiSetup::new(
        periph_spi1!(reg),
        thr.spi_1,
        pins,
        pclk2,
        BaudRate::Max(5_000_000),
    );
    let mut spi = SpiDrv::init(setup).into_master(miso_dma, mosi_dma);

    let mut chip = SpiChip::new_deselected(pin_cs);
    
    let systick = SysTickAlarmDrv::new(periph_sys_tick!(reg), thr.sys_tick);
    let alarm = Arc::new(AlarmDrv::new(systick.counter, systick.timer, SysTickTick));
    let eeprom = At250x0Drv::new(At250x0Kind::At25010b, alarm);

    const TMIB_2030: [u8; 48] = [
        0x74, 0x6D, 0x02, 0x63, 0x72, 0x6D, 0x33, 0x30, 0x39, 0x35, 0x30, 0x62, 0x2D, 0x31, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xEE, 0x07, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0xFB, 0x11, 0x00, 0x00, 0x99,
        0x00, 0x98, 0x9C,
    ];

    let mut buf = [0u8; 48];
    eeprom.read(&mut spi, &mut chip, 0x00, &mut buf).root_wait();

    eeprom
        .write(&mut spi, &mut chip, 0x00, &TMIB_2030)
        .root_wait()
        .unwrap();

    eeprom.read(&mut spi, &mut chip, 0x00, &mut buf).root_wait();

    // Enter a sleep state on ISR exit.
    reg.scb_scr.sleeponexit.set_bit();
}
