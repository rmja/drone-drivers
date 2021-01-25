#![feature(prelude_import)]
#![cfg_attr(not(feature = "std"), no_std)]

#[macro_use]
extern crate alloc;

mod adapters;
mod config;
pub mod configs;
pub mod controllers;
mod drivers;
mod drv;
mod opcode;
mod regs;
mod statusbyte;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Cc1200PartNumber {
    Cc1200,
    Cc1201,
}

pub use self::{
    adapters::{chip::Cc1200Chip, port::Cc1200Port, spi::Cc1200Spi},
    config::Cc1200Config,
    drv::{Cc1200Drv, TimeoutError},
    statusbyte::{State, StatusByte},
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
