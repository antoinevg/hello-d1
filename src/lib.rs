#![no_std]
#![allow(non_camel_case_types, unused)] // TODO

pub use d1_pac as pac;

pub mod audio_codec;
pub mod dmac;
pub mod logging;
pub mod plic;
pub mod timer;
