#![cfg_attr(rustfmt, rustfmt_skip)]

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{AUDIO_CODEC, CCU};

use crate::dmac::descriptor::Descriptor;
use crate::dmac::{self, descriptor, descriptor::DescriptorConfig, Dmac};
use crate::println;

/// Built-in audio codec
pub struct AudioCodec {
    audio_codec: AUDIO_CODEC,
}

impl AudioCodec {
    /// Create a new `AudioCodec`
    pub fn new(audio_codec: AUDIO_CODEC, ccu: &mut CCU) -> AudioCodec {
        // codec initialization
        init_codec_ccu(ccu);

        // configure sample rate, data transfer, open dac
        configure_dac(&audio_codec);

        Self { audio_codec }
    }

    /// Start `AudioCodec`
    pub fn start(&mut self, dmac: &mut Dmac, tx_buffer: &[u32]) {
        // configure dma and dma request
        let descriptor = configure_dma(&self.audio_codec, dmac, tx_buffer);

        // enable dac drq and dma
        enable_dac_drq_dma(&self.audio_codec, dmac, descriptor);

        // configure analog path
        configure_analog_path(&self.audio_codec);

        // test: enable debug mode
        //enable_debug_mode(&self.audio_codec);
    }

    /// Obtain a static `AudioCodec` instance for use in
    /// e.g. interrupt handlers
    ///
    /// # Safety
    ///
    /// 'Tis thine responsibility, that which thou doth summon.
    pub unsafe fn summon() -> Self {
        Self {
            audio_codec: d1_pac::Peripherals::steal().AUDIO_CODEC,
        }
    }
}

// ----------------------------------------------------------------------------

/// Enable debug mode
fn enable_debug_mode(audio_codec: &AUDIO_CODEC) {
    audio_codec.ac_dac_dg.write(|w| {
        w.dac_modu_select().debug()    // DAC_MODU_SELECT (0: Normal, 1: Debug)
         .dac_pattern_select().sin6()  // DAC_PATTERN_SELECT (0b00: Normal, 0b01: -6 dB Sine wave)
         .codec_clk_select().pll()     // CODEC_CLK_SELECT (0: PLL, 1: OSC for Debug)
         .da_swp().disable()           // DA_SWP  (0: Disabled, 1: Enabled)
         .adda_loop_mode().disable()   // ADDA_LOOP_MODE (0b000: Disabled, 0b001: ADC1/ADC2, 0b010: ADC3)
    });
}

/// 8.4.4.2 Playback Process, page 769-770
///
/// 1. Codec initialization: 1) configure AUDIO_CODEC_BGR_REG
///    to open the audio codec bus clock gating and de-assert bus
///    reset; 2) configure AUDIO_CODEC_DAC_CLK_REG and 3) PLL_AUDIO0_CTRL_REG
///    to configure PLL_Audio0 frequency and enable PLL_Audio0. For
///    details, refer to section 3.2 “CCU”.
/// ...
///
fn init_codec_ccu(ccu: &CCU) {
    // open audio codec bus clock gating and de-assert bus reset
    //
    // pg. 119 3.2.6.86 AUDIO_CODEC_BGR_REG
    // 0x0A5C AUDIO_CODEC Bus Gating Reset Register (Default Value: 0x0000_0000)
    //
    // 31:17  ---
    // 16     AUDIO_CODEC Reset            = 1 (0: Assert, 1: De-assert)
    // 15:01  ---
    // 00     Gating Clock for AUDIO_CODEC = 0 (0: Mask, 1: Pass)
    ccu.audio_codec_bgr.write(|w| {
        w.gating().pass()
         .rst().deassert()
    });

    // select PLL_Audio0 and open clock gating
    // pg. 117 3.2.6.84 AUDIO_CODEC_DAC_CLK_REG
    // 0x0A50 AUDIO_CODEC_DAC Clock Register (Default Value: 0x0000_0000)
    //
    // AUDIO_CODEC_DAC_CLK = Clock Source / M / N = PLL_AUDIO0 / 1 / 1 = PLL_AUDIO0
    //
    // 31     Gating Clock        = 1     (0: off, 1: on)
    // 30:27  ---
    // 26:24  Clock Source Select = 000   (00: PLL_AUDIO0 (1x), 01: PLL_AUDIO0 (DIV2), 10: PLL_AUDIO0 (DIV5))
    // 23:10  ---
    // 09:08  Factor N            = 00    (00: /1, 01: /2, 10: /4, 11: /8)
    // 07:05  ---
    // 04:00  Factor M            = 00000 (M = FACTOR_M + 1)
    ccu.audio_codec_dac_clk.write(|w| unsafe {
        w.clk_gating().on()
         .clk_src_sel().pll_audio0_1x()
         .factor_n().n1()
         .factor_m().bits(0)
    });

    // configure PLL_Audio0 frequency and enable PLL_Audio0
    //
    // pg. 64 3.2.6.7 PLL_AUDIO0_CTRL_REG
    // 0x0078 PLL_AUDIO0 Control Register (Default Value: 0x4814_5500)
    //
    // PLL_AUDIO0(1X) = (24MHz*N/M0/M1)/P/4 = (24000000 * 39 / 2 / 1) / 4 / 4 = 29 250 000
    //
    // 31     PLL Enable               = 1
    // 30     LDO Enable               = 1
    // 29:28  LOCK Enable + PLL Lock   = 11
    // 27     PLL Output Gate Enable   = 1
    // 26:25  ---                        00
    // 24     PLL SDM Enable           = 1
    // 23:22  ---                      = 00
    // 21:16  PLL Post-div P           = 00_0100   =  4
    // 15:08  PLL N                    = 0010_0111 = 39
    // 07:06  PLL Unlock Level         = 00        = 21-29 Clock Cycles
    // 05     PLL Lock Level           = 0         = 24-26 Clock Cycles
    // 04:02  ---
    // 01     PLL_INPUT_DIV_2          = 0         = M1 = PLL_INPUT_DIV2  + 1 = 1
    // 00     PLL_OUTPUT_DIV_2         = 1  L: 1   = M0 = PLL_OUTPUT_DIV2 + 1 = 2
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
    //
    // pg. 74 3.2.6.19 PLL_AUDIO0_PAT0_CTRL_REG
    // 0x0178 PLL_AUDIO0 Pattern0 Control Register (Default Value: 0x0000_0000)
    //
    // 31     Sigma-Delta Pattern Enable = 1
    // 30:29  Spread Frequency Mode      = 10 = Triangular (1-bit)
    // 28:20  Wave Step                  = 0_0000_0000
    // 19     SDM Clock Select           = 0  = 24 MHz
    // 18:17  Frequency                  = 00 = 31.5 kHz
    // 16:00  Wave Bottom                = 1_1110_1011_1000_0101 = 0x1EB85
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
}

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 2. Configure the sample rate and data transfer format, then open
///    the DAC.
/// ...
fn configure_dac(audio_codec: &AUDIO_CODEC) {
    // pg. 777 8.4.6.3 AC_DAC_FIFOC
    // 0x0010 DAC FIFO Control Register (Default Value: 0x0000_4000)
    //
    // 31:29  DAC_FS                 = x   (000: 48kHz,  010: 24kHz, 100: 12kHz,
    //                                      110: 192kHz, 001: 32kHz, 011: 16kHz, 101: 8kHz,
    //                                      111: 96kHz - Configure audio PLL for 44.1 etc.)
    // 28     FIR_VER                = 0   (0: 64-tap, 1: 32-tap)
    // 27     ---
    // 26     SEND_LASAT             = 1   (0: send zero, 1: send last sample)
    // 25:24  FIFO_MODE              = 01  (00: FIFO[19:0]=TXDAT[31:16], 01:FIFO[19:0]=TXDAT[15:0])
    // 23     ---
    // 22:21  DAC_DRQ_CLR_CNT        = 00  (00: WLEVEL > TXTL, 01: 4, 10: 8, 11: 16)
    // 20:15  ---
    // 14:08  TX_TRIG_LEVEL          = 00  (Default: 100_0000 = 0x40 = 64 = hald?)
    // 07     ---
    // 06     DAC_MONO_EN            = 00  (00: Stereo, 64 levels fifo, 01: 128 levels fifo)
    // 05     TX_SAMPLE_BITS         = 0   (0: 16 bits, 1: 20 bits)
    // 04     DAC_DRQ_EN             = 0
    // 03     DAC_IRQ_EN             = 1
    // 02     FIFO_UNDERRUN_IRQ_EN   = 1
    // 01     FIFO_OVERRUN_IRQ_EN    = 1
    // 00     FIFO_FLUSH             = 0   (0: self clear, 1: flush TX FIFO)
    audio_codec.ac_dac_fifoc.modify(|_, w| unsafe {
        w.dac_fs().fs48k()
         .send_last().zero()
         .fifo_mode().little_endian()
         .dac_mono_en().stereo()
         .tx_sample_bits().bits_20()
    });

    // Enable DAC digital part
    //
    // pg. 774 8.4.6.1 DAC Digital Part Control Register (Default Value: 0x0000_0000)
    // 0x0000 AC_DAC_DPC
    //
    // 31     EN_DAC  DAC Digital Part Enable
    // ...
    // 17:12  DVOL ATT = DVC * -1.16dB
    // ...
    // 00     HUB_EN  Audio Hub Enable
    audio_codec.ac_dac_dpc.modify(|_, w| unsafe {
        w.en_da().enable()
         .hpf_en().enable()
         .dvol().bits(0) // ATT = 0 * -1.16dB = 0dB
    });

    // Set DAC Volume
    //
    // pg. 775 8.4.6.2 DAC Volume Control Register (Default Value: 0x0000_A0A0)
    // 0x0004 DAC_VOL_CTRL
    //
    // 31:17  ---
    // 16     DAC_VOL_SEL - DAC Volume Control Selection Enable
    // 15:08  DAC_VOL_L  (0x00: Mute, 0x01: -119.25dB 0xa0: 0dB, 0xff: 71.25dB, 0.75dB / Step)
    // 07:00  DAC_VOL_R
    audio_codec.dac_vol_ctrl.modify(|_, w| unsafe {
        w.dac_vol_sel().enable()
         .dac_vol_l().bits(0xa0)
         .dac_vol_r().bits(0xa0)
    });
}

fn configure_analog_path(audio_codec: &AUDIO_CODEC) {
    // Enable left and right line out
    //
    // pg. 832 8.4.6.115 DAC Analog Control Register (Default Value: 0x0015_0000)
    // 0x0310 DAC_REG
    //
    // Linux: 0x0015c000  0000_0000_0001_0101_1100_0000_0000_0000
    //
    // 31:24  ---
    // 23     CURRENT_TEST_SELECT
    // 22     ---
    // 21:20  IOPVRS           = 01
    // 19:18  ILINEOUTAMPS     = 01
    // 17:16  IOPDACS          = 01
    // 15     DACL_EN          = 1
    // 14     DACR_EN          = 1
    // 13     LINEOUTLEN
    // 12     LMUTE            (0: Mute, 1: Unmute)
    // 11     LINEOUTREN
    // 10     RMUTE            (0: Mute, 1: Unmute)
    // 09:07  ---
    // 06     LINEOUTL_DIFFEN
    // 05     LINEOUTR_DIFFEN
    // 04:00  LINEOUT_VOL_CTRL (0x1f to 0x02, step=-1.5dB, mute = 0 or 1)
    audio_codec.dac.modify(|_, w| unsafe {
        w.iopvrs().c7u()
         .ilineoutamps().c7u()
         .iopdacs().c7u()
         .dacl_en().enable()
         .dacr_en().enable()
         .lineoutlen().disable()
         .lmute().unmute()
         .lineoutren().disable()
         .rmute().unmute()
         .lineout_vol_ctrl().bits(0x0)
    });

    // Enable headphone LDO regulator
    audio_codec.power.modify(|_, w| unsafe {
        w.aldo_en().enable()
         .hpldo_en().enable()
         .aldo_output_voltage().v180()
         .hpldo_output_voltage().v180()
         .bg_trim().bits(0b0001_1001)
    });

    // Enable ramp manual control
    audio_codec.ramp.modify(|_, w| unsafe {
        w.ramp_clk_div_m().bits(24)
         .rmc_en().enable()
    });

    // Enable headphone output
    //
    // pg. 844 8.4.6.121 Headphone2 Analog Control Register (Default Value: 0x0640_4000)
    // 0x0340 HP2_REG
    //
    // 31     HPFB_BUF_EN             = 1    Headphone feedback buffer op enable
    // 30:28  HEADPHONE GAIN          = 000  (0b000: 0dB, 0b001: -6dB ... 0b111: -42dB, -6dB step)
    // 27:26  HPFB_RES                = 01
    // 25:24  OPDRV_CUR               = 10   (00: Min, 01: Max)
    // 23:22  IOPHP                   = 01
    // 21     HP_DRVEN                = 1    Headphone Driver enable
    // 20     HP_DRVOUTEN             = 1    Headphone Driver Output Enable
    // 19     RSWITCH                 = 1
    // 18     RAMPEN                  = 0
    // 17     HPFB_IN_EN              = 1
    // 16     RAMP_FINAL_CONTROL      = 0
    // 15     RAMP_OUT_EN             = 1
    // 14:13  RAMP_FINAL_STATE_RES    = 10
    // 09:08  HPFB_BUF_OUTPUT_CURRENT = 00
    // 07:00  ---
    audio_codec.hp2.modify(|_, w| unsafe {
        w.hpfb_buf_en().enable()
         .headphone_gain().db0()
         .hpfb_res().r1000k()
         .opdrv_cur().bits(0b10)
         .iophp().c7u()
         .hp_drven().enable()
         .hp_drvouten().enable()
         .rswitch().vra1()
         .rampen().disable()
         .hpfb_in_en().enable()
         .ramp_final_control().select_ramp()
         .ramp_out_en().enable()
         .ramp_final_state_res().r10k()
         .hpfb_buf_output_current().i35()
    });
}


/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 3. Configure the DMA and DMA request.
/// ...
fn configure_dma(
    audio_codec: &AUDIO_CODEC,
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
        destination: audio_codec.ac_dac_txdata.as_ptr().cast(),
        dest_width: descriptor::DataWidth::Bit32,
        dest_addr_mode: descriptor::AddressMode::IoMode,
        dest_block_size: descriptor::BlockSize::Byte4,
        dest_drq_type: descriptor::DestDrqType::AudioCodec,
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

fn enable_dac_drq_dma(audio_codec: &AUDIO_CODEC, dmac: &mut Dmac, descriptor: descriptor::Descriptor) {
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

    // FIFO flush
    audio_codec.ac_dac_fifoc.modify(|_, w| w.fifo_flush().flush());

    // FIFO clear sample counter
    //
    // pg. 780 8.4.6. AC_DAC_CNT
    // 0x0024 DAC TX Counter Register (Default Value: 0x0000_0000)
    //
    // 31:00  TX_CNT TX Sample Counter
    audio_codec.ac_dac_cnt.write(|w| unsafe { w.bits(0) });

    // FIFO clear pending interrupts
    //
    // pg. 779 8.4.6.4 AC_DAC_FIFOS
    // 0x0013 DAC FIFO Status Register (Default Value: 0x0080_8008)
    //
    // 31:24  ---
    // 23     TX_EMPTY    (0: No room, 1: Room for more than one new sample)
    // 22:08  TXE_CNT Empty Space
    // 07:04  ---
    // 03     TXE_INT Empty Pending Interrupt
    // 02     TXU_INT Underrun Pending Interrupt
    // 01     TXO_INT Overrun Pending Interrupt
    // 00     ---
    audio_codec.ac_dac_fifos.write(|w| {
        w.txu_int().set_bit()
         .txo_int().set_bit()
    });

    // enable dac interrupts
    audio_codec.ac_dac_fifoc.modify(|_, w| {
        w.dac_drq_en().enable()
         .fifo_underrun_irq_en().enable()
         .fifo_overrun_irq_en().enable()
    });
}
