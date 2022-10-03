#![cfg_attr(rustfmt, rustfmt_skip)]

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{I2S_PCM2, CCU};

use crate::dmac::descriptor::Descriptor;
use crate::dmac::{self, descriptor, descriptor::DescriptorConfig, Dmac};
use crate::println;

/// I2s Peripheral
pub struct I2s {
    i2s_pcm2: I2S_PCM2,
}

impl I2s {
    /// Create a new `I2s`
    pub fn new(i2s_pcm2: I2S_PCM2, ccu: &mut CCU) -> I2s {
        // codec initialization
        //init_i2s_pcm2_ccu(ccu);

        // configure sample rate, data transfer, open dac
        //configure(&i2s_pcm2);

        Self { i2s_pcm2 }
    }

    /// Start `I2s`
    pub fn start(&mut self, dmac: &mut Dmac, tx_buffer: &[u32]) {
        // configure dma and dma request
        //let descriptor = configure_dma(&self.I2s, dmac, tx_buffer);

        // enable dac drq and dma
        //enable_dac_drq_dma(&self.I2s, dmac, descriptor);

        // configure analog path
        //configure_analog_path(&self.I2s);

        // test: enable debug mode
        //enable_debug_mode(&self.audio_codec);
    }

    /// Obtain a static `I2s` instance for use in
    /// e.g. interrupt handlers
    ///
    /// # Safety
    ///
    /// 'Tis thine responsibility, that which thou doth summon.
    pub unsafe fn summon() -> Self {
        Self {
            i2s_pcm2: d1_pac::Peripherals::steal().I2S_PCM2,
        }
    }
}
