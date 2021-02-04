#![feature(const_fn)]
#![feature(prelude_import)]
#![cfg_attr(not(feature = "std"), no_std)]

mod adapters;
mod aligned_chunks;
mod drivers;
mod drv;
mod opcode;

extern crate alloc;

pub use self::{
    adapters::chip::At250x0Chip,
    adapters::spi::At250x0Spi,
    drv::{At250x0Drv, At250x0Kind},
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
