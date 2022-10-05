#![cfg_attr(rustfmt, rustfmt_skip)]

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{CCU, GPIO, I2S_PCM2};

use crate::dmac::descriptor::Descriptor;
use crate::dmac::{self, descriptor, descriptor::DescriptorConfig, Dmac};
use crate::println;

/// I2s Peripheral
pub struct I2s {
    i2s_pcm2: I2S_PCM2,
}

impl I2s {
    /// Create a new `I2s`
    pub fn new(i2s_pcm2: I2S_PCM2, gpio: &GPIO, ccu: &mut CCU) -> I2s {
        // gpio initialization
        init_i2s_pcm2_gpio(gpio);

        // ccu initialization
        init_i2s_pcm2_ccu(ccu);

        // configure sample rate, data transfer
        configure_i2s_pcm2(&i2s_pcm2);

        Self { i2s_pcm2 }
    }

    /// Start `I2s`
    pub fn start(&mut self, dmac: &mut Dmac, tx_buffer: &[u32]) {
        // configure dma and dma request
        //let descriptor = configure_dma(&self.I2s, dmac, tx_buffer);

        // enable peripheral drq and dma
        //enable_drq_dma(&self.I2s, dmac, descriptor);

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

// ----------------------------------------------------------------------------

fn init_i2s_pcm2_gpio(gpio: &GPIO) {
    // configure gpio multiplex functions for I2S (Datasheet pg. 35)
    gpio.pb_cfg0.modify(|_, w| {
        w.pb3_select().i2_s2_din0()   // PB3 I2S2_DIN0
         .pb4_select().i2_s2_dout0()  // PB4 I2S2_DOUT0
         .pb5_select().i2_s2_bclk()   // PB5 I2S2_BCLK
         .pb6_select().i2_s2_lrck()   // PB6 I2S2_LRCK
         //.pb7_select().i2_s2_mclk()   // PB7 I2S2_MCLK - wm8731 devboard supplies own mclk
    });
}

fn init_i2s_pcm2_ccu(ccu: &CCU) {
    // configure PLL_Audio0 frequency and enable PLL_Audio0
    ccu.pll_audio0_ctrl.write(|w| unsafe {
        w.pll_en().enable()
         .pll_ldo_en().enable()
         .pll_output_gate().enable()
         .pll_sdm_en().enable()
         .pll_p().bits(4)
         .pll_n().bits(39)
         .pll_unlock_mdsel().cc_21_29()
         .pll_lock_mdsel().cc_24_26()
         .pll_input_div2().clear_bit()
         .pll_output_div2().set_bit()
    });

    // configure the pattern control register for PLL_Audio0
    ccu.pll_audio0_pat0_ctrl.write(|w| unsafe {
        w.sig_delt_pat_en().set_bit()
         .spr_freq_mode().triangular_1()
         .wave_step().bits(0x0)
         .sdm_clk_sel().f_24_m()
         .freq().f_31_5_k()
         .wave_bot().bits(0x1eb85)
    });

    // write PLL_AUDIOx Control Register[Lock Enable] to 0 and then to 1
    ccu.pll_audio0_ctrl.modify(|_, w| w.lock_enable().disable());
    ccu.pll_audio0_ctrl.modify(|_, w| w.lock_enable().enable());

    // wait for pll_audio0 lock
    while ccu.pll_audio0_ctrl.read().lock().is_unlocked() {
        println!("waiting for pll_audio0 lock");
    }

    // select PLL_Audio0 and open clock gating
    ccu.i2s_clk[2].write(|w| unsafe {
        w.clk_gating().on()
         .clk_src_sel().pll_audio0_1x()
         .factor_n().n1()
         .factor_m().bits(0)
    });

    // open i2s_pcm2 bus clock gating and de-assert bus reset
    ccu.i2s_bgr.modify(|_, w| unsafe {
        w.i2s2_gating().pass()
         .i2s2_rst().deassert()
    });
}

fn configure_i2s_pcm2(i2s_pcm2: &I2S_PCM2) {

}
