#![feature(prelude_import)]
#![cfg_attr(not(feature = "std"), no_std)]

#[macro_use]
extern crate alloc;

mod adapters;
pub mod controllers;
mod drv;
mod drivers;
mod opcode;
mod statusbyte;
mod config;
pub mod configs;
mod regs;

pub use self::{
    adapters::{chip::Cc1200Chip, port::Cc1200Port, spi::Cc1200Spi},
    drv::{Cc1200Drv, TimeoutError},
    statusbyte::{StatusByte, State},
    config::Cc1200Config
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
