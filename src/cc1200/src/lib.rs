#![feature(prelude_import)]
#![feature(bool_to_option)]
#![cfg_attr(not(feature = "std"), no_std)]

#[macro_use]
extern crate alloc;

mod adapters;
mod config;
pub mod configs;
pub mod controllers;
pub mod drivers;
mod drv;
mod opcode;
mod regs;
mod statusbyte;
mod gpio;
pub mod gpiocfg;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Cc1200PartNumber {
    Cc1200,
    Cc1201,
}

#[derive(Debug)]
pub struct Rssi(pub i8);

pub use self::{
    gpio::Cc1200Gpio,
    adapters::{
        chip::Cc1200Chip,
        port::Cc1200Port,
        spi::Cc1200Spi,
        timer::{Cc1200Timer, Cc1200TimerPin},
        uptime::Cc1200Uptime,
    },
    config::Cc1200Config,
    drv::{Cc1200Drv, TimeoutError, RxFifoOverflowError, TxFifoUnderflowError},
    statusbyte::{State, StatusByte},
    opcode::Strobe,
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
