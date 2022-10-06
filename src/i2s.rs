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
    pub fn new(i2s_pcm2: I2S_PCM2, gpio: &GPIO, ccu: &CCU) -> I2s {
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
        let descriptor = configure_dma(&self.i2s_pcm2, dmac, tx_buffer);

        // enable peripheral drq and dma
        enable_drq_dma(&self.i2s_pcm2, dmac, descriptor);
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

/// Configure gpio multiplex functions for I2S (Datasheet pg. 35)
fn init_i2s_pcm2_gpio(gpio: &GPIO) {
    gpio.pb_cfg0.modify(|_, w| {
        w.pb3_select().i2_s2_din0()   // PB3 I2S2_DIN0
         .pb4_select().i2_s2_dout0()  // PB4 I2S2_DOUT0
         .pb5_select().i2_s2_bclk()   // PB5 I2S2_BCLK
         .pb6_select().i2_s2_lrck()   // PB6 I2S2_LRCK
         //.pb7_select().i2_s2_mclk()   // PB7 I2S2_MCLK - wm8731 devboard supplies own mclk
    });
}

/// Set up I2S clocks
///
/// TODO Make pll_audio config optional for when codec has its own clock
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
    //
    // TODO check if manual is correct in asserting this comes after clock config
    ccu.i2s_bgr.modify(|_, w| unsafe {
        w.i2s2_gating().pass()
         .i2s2_rst().deassert()
    });
}

fn configure_i2s_pcm2(i2s_pcm2: &I2S_PCM2) {
    // disable i2c_pcm2
    i2s_pcm2.i2s_pcm_ctl.modify(|_, w| {
        w.gen().disable()
         .txen().disable()
         .rxen().disable()
    });

    // set fifo to mode1 (little-endian)
    i2s_pcm2.i2s_pcm_fctl.modify(|_, w| {
        w.txim().mode1()
    });

    // flush tx/rx fifo
    i2s_pcm2.i2s_pcm_fctl.modify(|_, w| {
        w.ftx().flush()
         .frx().flush()
    });

    // clear tx/rx fifo counters
    i2s_pcm2.i2s_pcm_txcnt.write(|w| unsafe { w.tx_cnt().bits(0) });
    i2s_pcm2.i2s_pcm_rxcnt.write(|w| unsafe { w.rx_cnt().bits(0) });

    // configure i2c_pcm2 mode: external clock, i2s mode
    i2s_pcm2.i2s_pcm_ctl.modify(|_, w| {
        w
            .rx_sync_en_start().enable() // only takes effect if rx_sync_en is high
            .rx_sync_en().enable()       // rx_sync: enable
            .bclk_out().input()          // i2s_clock_external
            .lrck_out().input()          // i2s_clock_external
            .dout0_en().enable()
            .out_mute().normal()
            .mode_sel().left()           // Offset 0: Left-Justified, Offset 1: I2S
            .loopback().normal()         // Set to test for loopback test
    });
    i2s_pcm2.i2s_pcm_clkd.modify(|_, w| {
        w
            .mclko_en().disable()  // disable mclk output
            .mclkdiv().div_1()     // set mclk div ratio from pll_audio
            .bclkdiv().div_1()     // set bclk div ratio from pll_audio
    });

    // configure i2c_pcm2 format:
    i2s_pcm2.i2s_pcm_fmt0.modify(|_, w| unsafe {
        w
            .lrck_polarity().high()       // low: left, high: right - I2S_CHANNEL_FMT_RIGHT_LEFT
            .lrck_period().bits(31)       // period = sr - 1
            .sr().bits_32()               // sample resolution: 32 bit
            .sw().bits_32()               // slot width: 32 bit
            .blck_polarity().normal()     // normal: negative, invert: positive
            .edge_transfer().alternate()  // in conjunction with blck_polarity sets pos/neg edge

    });
    i2s_pcm2.i2s_pcm_fmt1.modify(|_, w| {
        w
            .rx_mls().msb()    // i2s mode
            .tx_mls().msb()    // i2s mode
            .sext().zero()     // if sampleres < slot width
            .rx_pdm().linear() // linear PCM / u-law / A-law
            .tx_pdm().linear() // linear PCM / u-law / A-law
    });
    i2s_pcm2.i2s_pcm_chcfg.modify(|_, w| unsafe {
        w.tx_slot_hiz().normal()  // defaults - only used for TDM
         .tx_state().zero()       // defaults - only used for TDM
         .tx_slot_num().bits(1)   // DMA/FIFO num slots = n + 1
        //.rx_slot_num().bits(0) // TODO
    });
    i2s_pcm2.i2s_pcm_tx0chsel.modify(|_, w| unsafe {
        w.offset().bits(1) // I2S: 1
         .chsel().bits(1)  // Num slots = n + 1
         .chen().bits(0b0000_0000_0000_0011) // Enable slots 0 & 1
    });
    i2s_pcm2.i2s_pcm_tx0chmap1.modify(|_, w| unsafe {
        w.ch0_map().bits(0)
         .ch1_map().bits(1)
    });
    /*i2s_pcm2.i2s_pcm_rxchsel.modify(|_, w| {
        w
    });
    i2s_pcm2.i2s_pcm_rxchmap0.modify(|_, w| {
        w
    });*/


}

/// Configure DMA
///
/// TODO configure DMA for incoming audio as well
fn configure_dma(
    i2s_pcm2: &I2S_PCM2,
    dmac: &mut Dmac,
    tx_buffer: &[u32],
) -> descriptor::Descriptor {
    // create dma descriptor
    let descriptor_config = descriptor::DescriptorConfig {
        // source
        source: tx_buffer.as_ptr().cast(),
        byte_counter: tx_buffer.len() * core::mem::size_of::<u32>(),
        src_data_width: descriptor::DataWidth::Bit32,
        src_addr_mode: descriptor::AddressMode::LinearMode,
        src_block_size: descriptor::BlockSize::Byte4,
        src_drq_type: descriptor::SrcDrqType::Dram,
        // destination
        destination: i2s_pcm2.i2s_pcm_txfifo.as_ptr().cast(),
        dest_width: descriptor::DataWidth::Bit32,
        dest_addr_mode: descriptor::AddressMode::IoMode,
        dest_block_size: descriptor::BlockSize::Byte4,
        dest_drq_type: descriptor::DestDrqType::I2sPcm2Tx,
        // config
        link: None,
        wait_clock_cycles: 0,
        bmode: descriptor::BModeSel::Normal,
    };
    let mut descriptor: Descriptor = descriptor_config.try_into().unwrap();

    // link descriptor to itself
    let descriptor_ptr = &descriptor as *const Descriptor as *const ();
    descriptor.link = descriptor_ptr as u32;
    descriptor.link |= ((descriptor_ptr as usize >> 32) as u32) & 0b11;

    descriptor
}

fn enable_drq_dma(i2s_pcm2: &I2S_PCM2, dmac: &mut Dmac, descriptor: descriptor::Descriptor) {
    unsafe {
        // enable dma half/package interrupts
        dmac.irq_en0().write(|w| {
            w.dma0_hlaf_irq_en().set_bit()
             .dma0_pkg_irq_en().set_bit()
        });

        dmac.channels[0]
            .set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Wait);
        dmac.channels[0].start_descriptor(NonNull::from(&descriptor));
    }

    // FIFO clear pending interrupts
    i2s_pcm2.i2s_pcm_ista.write(|w| {
        w.txu_int().set_bit()
         .txo_int().set_bit()
         .txe_int().set_bit() // TODO don't need this
    });

    // enable i2s_pcm2 interrupts
    i2s_pcm2.i2s_pcm_int.write(|w| {
        w.tx_drq().enable()
         .txui_en().enable()
         .txoi_en().enable()
         .txei_en().enable() // TODO don't need this
         // TODO rx_drq etc.
    });

    // enable i2c_pcm2
    i2s_pcm2.i2s_pcm_ctl.modify(|_, w| {
        w.txen().enable()
         .rxen().enable()
    });
    i2s_pcm2.i2s_pcm_ctl.modify(|_, w| {
        w.gen().enable()
    });

}
