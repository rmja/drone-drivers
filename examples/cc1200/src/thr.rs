//! The threads.

pub use drone_cortexm::thr::{init, init_extended};
pub use drone_stm32_map::thr::*;

use drone_cortexm::thr;

thr::nvic! {
    /// Thread-safe storage.
    thread => pub Thr {};

    /// Thread-local storage.
    local => pub ThrLocal {};

    /// Vector table.
    vtable => pub Vtable;

    /// Thread token set.
    index => pub Thrs;

    /// Threads initialization token.
    init => pub ThrsInit;

    threads => {
        exceptions => {
            /// All classes of faults.
            pub hard_fault;
            pub sys_tick;
        };
        interrupts => {
            // Vector table for stm32f429 is in PM0090 table 62 page 375.
            5: pub rcc;
            16: pub dma1_ch5; // USART2_RX: DMA1, stream 5 (channel 4).
            17: pub dma1_ch6; // USART2_TX: DMA1, stream 6 (channel 4).
            23: pub exti9_5;
            28: pub tim2;
            30: pub tim4;
            35: pub spi1;
            38: pub usart2;
            58: pub dma2_ch2; // SPI1_RX: DMA2, stream 2 (channel 3).
            59: pub dma2_ch3; // SPI1_TX: DMA2, stream 3 (channel 3).

            19: pub rf; // CAN1_TX
            20: pub framesync;
        }
    };
}
