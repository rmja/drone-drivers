//! The root task.

use core::{pin::Pin, task::Poll};

use crate::{consts, thr, thr::ThrsInit, Regs, tasks};
use alloc::sync::Arc;
use drone_cc1200_drv::{Cc1200Drv, Cc1200Gpio, configs::{CC1200_WMBUS_MODECMTO_FULL_INFINITY, CC1200_WMBUS_MODECMTO_FULL_PACKET}, controllers::{debug::DebugController, infinite::InfiniteController, packet::PacketController}};
use drone_core::{fib, log, sync::Mutex};
use drone_cortexm::{drv::sys_tick::SysTick, periph_sys_tick, processor::spin, reg::prelude::*, swo, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::{
        periph_dma1,periph_dma1_ch5,periph_dma1_ch6,
        periph_dma2, periph_dma2_ch2, periph_dma2_ch3
    },
    exti::periph_exti6,
    gpio::{
        periph_gpio_a2, periph_gpio_a3, periph_gpio_a5, periph_gpio_a6, periph_gpio_a7, periph_gpio_a_head, periph_gpio_b0,
        periph_gpio_b8, periph_gpio_b_head, periph_gpio_d12, periph_gpio_d_head, periph_gpio_i1,periph_gpio_b10,
        periph_gpio_h10,
        periph_gpio_i3,
        periph_gpio_i_head,periph_gpio_h_head,
    },
    spi::periph_spi1,
    tim::{periph_tim2, periph_tim4},
    uart::periph_usart2,
};
use drone_stm32f4_hal::{
    dma::{config::*, DmaCfg},
    exti::{periph_syscfg, prelude::*, ExtiDrv, Syscfg},
    gpio::{prelude::*, GpioHead},
    rcc::{prelude::*, periph_flash, periph_pwr, periph_rcc, Flash, Pwr, Rcc, RccSetup},
    spi::{self, chipctrl::*, prelude::*},
    tim::{prelude::*, GeneralTimCfg, GeneralTimSetup},
    uart::{self, prelude::*}
};
use drone_time::{prelude::*, AlarmDrv, TimeSpan, UptimeDrv};
use futures::{Future, future::{self, Either}};
use futures::prelude::*;

const EXAMPLE_TYPE: u32 = 0;

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
    thr.exti9_5.enable_int();
    thr.tim2.enable_int();
    thr.tim4.enable_int();
    thr.spi1.enable_int();
    thr.usart2.enable_int();
    thr.dma1_ch5.enable_int();
    thr.dma1_ch6.enable_int();
    thr.dma2_ch2.enable_int();
    thr.dma2_ch3.enable_int();

    thr.rf.enable_int();
    thr.framesync.enable_int();
    thr.forwarder.enable_int();

    // Let app threads have lowest priority
    thr.rf.set_priority(15 << 4);
    thr.framesync.set_priority(15 << 4);
    thr.forwarder.set_priority(15 << 4);

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
    let gpio_h = GpioHead::with_enabled_clock(periph_gpio_h_head!(reg));
    let gpio_i = GpioHead::with_enabled_clock(periph_gpio_i_head!(reg));

    // Configure SPI GPIO pins.
    let sck_pin = gpio_a
        .pin(periph_gpio_a5!(reg))
        .into_alternate()
        .into_pushpull()
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
        .with_speed(GpioPinSpeed::MediumSpeed);
    let cs_pin = gpio_b
        .pin(periph_gpio_b0!(reg))
        .into_output()
        .into_pushpull()
        .into_pullup()
        .with_speed(GpioPinSpeed::MediumSpeed);

    // Configure UART GPIO pins.
    let pin_uart_tx = gpio_a
        .pin(periph_gpio_a2!(reg))
        .into_alternate()
        .into_pushpull()
        .with_speed(GpioPinSpeed::MediumSpeed);
    let pin_uart_rx = gpio_a
        .pin(periph_gpio_a3!(reg))
        .into_alternate()
        .into_pushpull()
        .with_speed(GpioPinSpeed::MediumSpeed);

    let miso_pin_reset = unsafe { miso_pin.clone() };

    // Thread work indicator pins
    let forwarder_busy = gpio_b
        .pin(periph_gpio_b10!(reg))
        .into_output()
        .into_pushpull();
    let framesync_busy = gpio_h
        .pin(periph_gpio_h10!(reg))
        .into_output()
        .into_pushpull();
    let rf_busy = gpio_i
        .pin(periph_gpio_i3!(reg))
        .into_output()
        .into_pushpull();


    // Initialize exti.
    let syscfg = Syscfg::with_enabled_clock(periph_syscfg!(reg));
    let exti6 = Arc::new(ExtiDrv::new(periph_exti6!(reg), thr.exti9_5, &syscfg).into_falling_edge());

    // Initialize dma.
    let dma2 = DmaCfg::with_enabled_clock(periph_dma2!(reg));
    let miso_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch2!(reg), thr.dma2_ch2));
    let mosi_dma = dma2.ch(DmaChSetup::new(periph_dma2_ch3!(reg), thr.dma2_ch3));

    // Initialize spi.
    let pins = spi::SpiPins::default()
        .sck(sck_pin)
        .miso(miso_pin)
        .mosi(mosi_pin);
    let setup = spi::SpiSetup::new(
        periph_spi1!(reg),
        thr.spi1,
        pins,
        pclk2,
        spi::BaudRate::Max(7_700_000),
    );
    let mut spi = spi::SpiDrv::init(setup).into_master(miso_dma, mosi_dma);

    // Initialize uart dma.
    let dma1 = DmaCfg::with_enabled_clock(periph_dma1!(reg));
    // let uart_rx_dma = dma1.ch(DmaChSetup::new(periph_dma1_ch5!(reg), thr.dma1_ch5));
    let uart_tx_dma = dma1.ch(DmaChSetup::new(periph_dma1_ch6!(reg), thr.dma1_ch6));

    // Initialize uart.
    let uart_pins = uart::UartPins::default()
        .tx(pin_uart_tx)
        .rx(pin_uart_rx);
    let mut setup = uart::UartSetup::init(periph_usart2!(reg), thr.usart2, pclk1);
    setup.baud_rate = uart::BaudRate::Nominal(115200);
    let mut uart_tx_drv = uart::UartDrv::init(setup).into_tx(uart_tx_dma,  &uart_pins);

    let mut cc1200_cs = SpiChip::new_deselected(cs_pin);

    // Deselect other SPI devices on same bus
    SpiChip::new_deselected(
        gpio_i
            .pin(periph_gpio_i1!(reg))
            .into_output()
            .into_pushpull()
            .into_pullup()
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
        thr.tim2,
        pclk1,
        TimFreq::Nominal(consts::TIM2_FREQ),
    ))
    .into_count_up()
    .ch1(|ch| ch.into_output_compare())
    .into_master();

    let mut tim4 = GeneralTimCfg::with_enabled_clock(GeneralTimSetup::new(
        periph_tim4!(reg),
        thr.tim4,
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
        thr.tim2,
        consts::Tim2Tick,
    );

    let alarm = Arc::new(AlarmDrv::new(tim2.counter, tim2.ch1, consts::Tim2Tick));

    reset_pin.set();
    alarm.sleep(TimeSpan::from_millis(100)).await;
    reset_pin.clear();
    alarm.sleep(TimeSpan::from_millis(100)).await;
    reset_pin.set();
    alarm.sleep(TimeSpan::from_millis(100)).await;
    reset_pin.clear();

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

    if EXAMPLE_TYPE == 0 {
        // TASK BASED EXAMPLE

        let (rf_tx, rf_rx) = drone_core::sync::spsc::ring::channel(10);
        let (packet_tx, packet_rx) = drone_core::sync::spsc::ring::channel(10);

        thr.forwarder.exec_factory(move || tasks::forwarder(packet_rx, uart_tx_drv, forwarder_busy));
        thr.framesync.exec_factory(move || tasks::framesync(rf_rx, packet_tx, framesync_busy));
        thr.rf.exec_factory(move || tasks::rf(port, alarm.clone(), spi.clone(), cc1200_cs, tim4.ch1, uptime, rf_tx, rf_busy));
    }
    else {
        // SIMPLE EXAMPLES BELOW
        let mut cc1200 = Cc1200Drv::init(port, alarm.clone());

        // Issue the hardware reset sequence
        cc1200.hw_reset(&mut cc1200_cs).await.unwrap();

        // DEBUG CONTROLLER EXAMPLE
        // let mut debug = DebugController::setup(cc1200, spi.clone(), cc1200_cs, &CC1200_WMBUS_MODECMTO_FULL_INFINITY).await.unwrap();
        // debug.tx_unmodulated().await;
        // alarm.sleep(TimeSpan::from_secs(3)).await;
        // debug.tx_modulated_01().await;
        // alarm.sleep(TimeSpan::from_secs(3)).await;
        // debug.tx_modulated_pn9().await;
        // alarm.sleep(TimeSpan::from_secs(3)).await;
        // debug.idle().await;
        // let (cc1200, cc1200_cs) = debug.release();


        // INFINITE CONTROLLER EXAMPLE
        // let mut infinite = InfiniteController::setup(
        //     cc1200,
        //     spi.clone(),
        //     cc1200_cs,
        //     tim4.ch1,
        //     uptime.clone(),
        //     &CC1200_WMBUS_MODECMTO_FULL_INFINITY,
        //     Cc1200Gpio::Gpio0,
        // )
        // .await
        // .unwrap();
        // let mut count: i32 = 0;
        // let mut chunk_stream = infinite.receive().await;
        // while let Some(chunk) = chunk_stream.next().await {
        //     // println!("{:?}", chunk.upstamp);
        //     count += 1;

        //     if count == 10000 {
        //         break;
        //     }
        // }
        // drop(chunk_stream);
        // infinite.idle().await;
        // let (cc1200, cc1200_cs, tim4ch1) = infinite.release();


        // PACKET CONTROLLER EXAMPLE
        let mut packet = PacketController::setup(
            cc1200,
            spi.clone(),
            cc1200_cs,
            tim4.ch1,
            uptime.clone(),
            &CC1200_WMBUS_MODECMTO_FULL_PACKET,
            Cc1200Gpio::Gpio0,
        )
        .await
        .unwrap();

        // let tx: Vec<u8> = (0..=255).collect();
        // for length in 1..=256 {
        //     packet.write(&tx[0..length]).await;
        //     let before = uptime.now();
        //     packet.transmit().await.unwrap();
        //     let after = uptime.now();
        //     let tx_us = (after - before).as_micros();
        //     println!("Tx strobe + actual transmission took {}us", tx_us);
        // }

        // Let the receiver run until 10 seconds, or until 5 seconds after the last received packet.
        let mut stop = alarm.sleep(TimeSpan::from_secs(120));
        let mut packet_stream = packet.receive_variable_length().await;
        // The receiver is started, start waiting for packets.
        while let Either::Left(packet) = future::select(packet_stream.next(), stop).await {
            stop = alarm.sleep(TimeSpan::from_secs(60));
        }

        println!("Rx completed");
    }

    // Enter a sleep state on ISR exit.
    reg.scb_scr.sleeponexit.set_bit();
}
