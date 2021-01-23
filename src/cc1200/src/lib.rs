#![feature(prelude_import)]
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod adapters;
mod controllers;
mod drv;
mod opcode;

pub use self::{
    adapters::{chip::Cc1200Chip, port::Cc1200Port, spi::Cc1200Spi},
    drv::Cc1200Drv,
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
