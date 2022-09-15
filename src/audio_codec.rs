#![cfg_attr(rustfmt, rustfmt_skip)]

/// 8.4 Audio Codec, page 758
///
/// The clock source for the digital part is the PLL_AUDIO0 and
/// PLL_AUDIO1.
///
/// For the DAC clock, configure AUDIO_CODEC_DAC_CLK_REG[25:24] to
/// select the clock source.
///
/// The PK-PK jitter of PLL_AUDIO0 and PLL_AUDIO1 should be less than
/// 200 ps.
///
/// The stereo DAC sample rate can be configured by setting the
/// register. To save power, the analog DACL can be enabled or
/// disabled by setting the bit[15] of the DAC_REG register, and the
/// analog DACR can be enabled or disabled by setting the bit[14] of
/// the DAC_REG register. The digital DAC part can be enabled or
/// disabled by the bit[31] of the AC_DAC_DPC register.
///
/// 8.4.4.2 Playback Process, page 769-770
/// 1. Codec initialization: configure AUDIO_CODEC_BGR_REG to open the
///    audio codec bus clock gating and de-assert bus reset; configure
///    AUDIO_CODEC_DAC_CLK_REG and PLL_AUDIO0_CTRL_REG to configure
///    PLL_Audio1 frequency and enable PLL_Audio1. For details, refer
///    to section 3.2 “CCU”.
/// 2. Configure the sample rate and data transfer format, then open
///    the DAC.
/// 3. Configure the DMA and DMA request.
/// 4. Enable the DAC DRQ and DMA.
///
/// Linux Audio: https://wiki.sipeed.com/hardware/en/lichee/RV/user.html
/// Dump from Linux: `busybox devmem 0x02030000`
///
/// CCU 0x02001000
/// AUDIO_CODEC_BGR_REG       0x02001a5c  0x00010001  ..
/// AUDIO_CODEC_DAC_CLK_REG   0x02001a50  0x80000003  0x80000000
/// PLL_AUDIO0_CTRL_REG       0x02001078  0xF8001D01  0xf9042701
/// PLL_AUDIO0_PAT0_CTRL_REG  0x02001178  0x4001288D  0xc001eb85
/// PLL_AUDIO0_PAT1_CTRL_REG  0x0200117c  0x00000000  ..
/// PLL_AUDIO0_BIAS_REG       0x02001378  0x00030000  ..
///
/// AUDIO_CODEC 0x02030000
/// AC_DAC_DPC                0x02030000  0x00000000  0x80000000
/// DAC_VOL_CTRL              0x02030004  0x0001A0A0  ..
/// AC_DAC_FIFOC              0x02030010  0x03004000  ..
/// DAC_REG                   0x02030310  0x0015007A  0x0015c000
/// RAMP_REG                  0x0203031c  0x00180000  0x00180002
/// BIAS_REG                  0x02030320  0x0000007C  ..
/// HP2_REG                   0x02030340  0x76404000  0x867ac000
/// POWER_REG                 0x02030348  0x80013319  0xc0013319
///
/// Manual:
///
/// busybox devmem 0x02030000 w 0x80000000
/// busybox devmem 0x02030348 w 0xC0013319
/// busybox devmem 0x0203031c w 0x00180002


use core::ptr::NonNull;

use d1_pac as pac;
use pac::{AUDIO_CODEC, CCU};

use crate::dmac::descriptor::Descriptor;
use crate::dmac::{self, descriptor, descriptor::DescriptorConfig, Dmac};
use crate::println;

/// Built-in audio codec
pub struct AudioCodec {
    audio_codec: AUDIO_CODEC,
    dma_descriptor: Option<descriptor::Descriptor>,
}

impl AudioCodec {
    /// Create a new `AudioCodec`
    pub fn new(audio_codec: AUDIO_CODEC, ccu: &mut CCU) -> AudioCodec {
        // 1. codec initialization
        init_codec_ccu(ccu);

        // 2. configure sample rate, data transfer, open dac
        configure_dac(&audio_codec);

        Self {
            audio_codec,
            dma_descriptor: None,
        }
    }

    pub fn start(&mut self, dmac: &mut Dmac) {
        // 3. configure dma and dma request
        let descriptor = configure_dma(&self.audio_codec, dmac);
        self.dma_descriptor = Some(descriptor);

        // 4. enable dac drq and dma
        enable_dac_drq_dma(&self.audio_codec, dmac, self.dma_descriptor.as_ref().unwrap());

        // configure analog path
        configure_analog_path(&self.audio_codec);

        // test: enable debug mode
        //enable_debug_mode(&self.audio_codec);
    }

    /// Obtain a static `AudioCodec` instance for use in
    /// e.g. interrupt handlers
    ///
    /// TODO populate dma descriptors
    ///
    /// # Safety
    ///
    /// 'Tis thine responsibility, that which thou doth summon.
    pub unsafe fn summon() -> Self {
        Self {
            audio_codec: d1_pac::Peripherals::steal().AUDIO_CODEC,
            dma_descriptor: None,
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
///    to configure PLL_Audio1 frequency and enable PLL_Audio1. For
///    details, refer to section 3.2 “CCU”.
/// ...
///
/// Also see: d1-sdk.repo/lichee/brandy-2.0/u-boot-2018/drivers/sound/sun8iw18-codec.c
///           sunxi_codec_init()
///
fn init_codec_ccu(ccu: &CCU) {
    // 1. open audio codec bus clock gating and de-assert bus reset
    //
    // pg. 119 3.2.6.86 AUDIO_CODEC_BGR_REG
    // 0x0A5C AUDIO_CODEC Bus Gating Reset Register (Default Value: 0x0000_0000)
    //
    // 31:17  ---
    // 16     AUDIO_CODEC Reset            = 1 (0: Assert, 1: De-assert)
    // 15:01  ---
    // 00     Gating Clock for AUDIO_CODEC = 0 (0: Mask, 1: Pass)
    //
    // xplayer: 0x00010001  0000_0000_0000_0001_0000_0000_0000_0001
    ccu.audio_codec_bgr.write(|w| {
        w.gating().pass()
         .rst().deassert()
    });

    // 2. configure PLL_Audio0 frequency and select PLL_Audio0
    // pg. 117 3.2.6.84 AUDIO_CODEC_DAC_CLK_REG
    // 0x0A50 AUDIO_CODEC_DAC Clock Register (Default Value: 0x0000_0000)
    //
    // xplayer: 0x80000000  1000_0000_0000_0000_0000_0000_0000_0000
    //
    // AUDIO_CODEC_DAC_CLK = Clock Source / M / N = PLL_AUDIO0 / 1 / 1 = PLL_AUDIO0
    //
    // 31     Gating Clock        = 1     (0: off, 1: on)
    // 30:27  ---
    // 26:24  Clock Source Select = 000   (00: PLL_AUDIO0 (1x), 01: PLL_AUDIO0 (DIV2), 10: PLL_AUDIO0 (DIV5))
    // 23:10  ---
    // 09:08  Factor N            = 00    (00: /1, 01: /2, 10: /4, 11: /8)
    // 07:05  ---
    // 04:00  Factor M            = 00011 (M = FACTOR_M + 1)
    ccu.audio_codec_dac_clk.write(|w| unsafe {
        w.clk_gating().on()
         .clk_src_sel().pll_audio0_1x()
         .factor_n().n1()
         .factor_m().bits(0)
    });

    // 3. configure PLL_Audio0 frequency and enable PLL_Audio0
    //
    // pg. 64 3.2.6.7 PLL_AUDIO0_CTRL_REG
    // 0x0078 PLL_AUDIO0 Control Register (Default Value: 0x4814_5500)
    // Default:               0100_1000_0001_0100_0101_0101_0000_0000
    // xplayer:   0xf9042701  1111_1001_0000_0100_0010_0111_0000_0001
    //
    // PLL_AUDIO0(1X) = (24MHz*N/M0/M1)/P/4 = (24000000 * 85 / 1 / 1) / 20 / 4 = 25 500 000
    //                                      = (24000000 * 39 / 1 / 2) / 19 / 4 =
    //
    // 31     PLL Enable               = 1
    // 30     LDO Enable               = 1
    // 29:28  LOCK Enable + PLL Lock   = 11
    // 27     PLL Output Gate Enable   = 1
    // 26:25  ---                        00
    // 24     PLL SDM Enable           = 1         = Enable spread spectrum and decimal division
    // 23:22  ---                      = 00
    // 21:16  PLL Post-div P           = 00_0100   = D: 20   L: 4
    // 15:08  PLL N                    = 0010_0111 = D: 85   L: 39
    // 07:06  PLL Unlock Level         = 00        = 21-29 Clock Cycles
    // 05     PLL Lock Level           = 0         = 24-26 Clock Cycles
    // 04:02  ---
    // 01     PLL_INPUT_DIV_2          = 0         = M1 = PLL_INPUT_DIV2  + 1 = 1 // D1
    // 00     PLL_OUTPUT_DIV_2         = 1  L: 1   = M0 = PLL_OUTPUT_DIV2 + 1 = 1 // D2
    ccu.pll_audio0_ctrl.modify(|_, w| unsafe {
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

    // pg. 74 3.2.6.19 PLL_AUDIO0_PAT0_CTRL_REG
    // 0x0178 PLL_AUDIO0 Pattern0 Control Register (Default Value: 0x0000_0000)
    //
    // xplayer: 0xc001eb85  1100_0000_0000_0001_1110_1011_1000_0101
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
         .wave_bot().bits(0b1_1110_1011_1000_0101)
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
///
/// Also see: sunxi_codec_hw_params()
///           sunxi_codec_playback_prepare()
fn configure_dac(audio_codec: &AUDIO_CODEC) {
    // pg. 777 8.4.6.3 AC_DAC_FIFOC
    // 0x0010 DAC FIFO Control Register (Default Value: 0x0000_4000)
    // Default: 0000_0000_0000_0000_0100_0000_0000_0000
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
         .tx_sample_bits().bits_16()
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
    // 00     HUB_EN  Audio Hub Enable  ?????
    audio_codec.ac_dac_dpc.modify(|_, w| unsafe {
        w.en_da().enable()
         //.hpf_en().enable()
         //.dvol().bits(6) // ATT = 0 * -1.16dB = 0dB
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
         //.dac_vol_l().bits(0x98)
         //.dac_vol_r().bits(0x98)
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
    // 0xc0013319  1100_0000_0000_0001_0011_0011_0001_1001
    audio_codec.power.modify(|_, w| unsafe {
        w.aldo_en().enable()
         .hpldo_en().enable()
         .aldo_output_voltage().v180()
         .hpldo_output_voltage().v180()
         .bg_trim().bits(0b0001_1001) // ?? default is 0x25
    });

    // Enable ramp manual control
    // 0x00180002  0000_0000_0001_1000_0000_0000_0000_0010
    audio_codec.ramp.modify(|_, w| unsafe {
        w.ramp_clk_div_m().bits(24) // 20:16 0b11000
         .rmc_en().enable()
    });

    // Enable headphone output
    //
    // pg. 844 8.4.6.121 Headphone2 Analog Control Register (Default Value: 0x0640_4000)
    // 0x0340 HP2_REG
    //
    // xplayer: 0x867ac000  1000_0110_0111_1010_1100_0000_0000_0000
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

    // Turn on speaker ???
    // looks like this is actually NC on the RV Dock
    /*let gpio = unsafe { pac::Peripherals::steal().GPIO };
    const PB12: u32 = 12;
    gpio.pb_cfg1.modify(|_, w| w.pb12_select().output());
    gpio.pb_dat.modify(|r, w| unsafe {
        w.pb_dat().bits(r.pb_dat().bits() | (0x1 << PB12))
    });*/
}

// Audio Buffers
//
// TODO pg. 225  make sure that TX_BUFFER_1 is word-aligned
pub const BLOCK_LENGTH: usize = 128;
pub const STEREO_BLOCK_LENGTH: usize = BLOCK_LENGTH * 2; // 2 channels
pub const TX_BUFFER_LENGTH: usize = STEREO_BLOCK_LENGTH;
pub static mut TX_BUFFER: [u32; TX_BUFFER_LENGTH] = [0; TX_BUFFER_LENGTH];

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 3. Configure the DMA and DMA request.
/// ...
fn configure_dma(
    audio_codec: &AUDIO_CODEC,
    dmac: &mut Dmac
) -> descriptor::Descriptor {
    // configure dma descriptor
    let ac_dac_txdata = &unsafe { &*AUDIO_CODEC::PTR }.ac_dac_txdata as *const _ as *mut ();
    let source = unsafe { TX_BUFFER.as_ptr().cast() };
    let byte_counter = unsafe { TX_BUFFER.len() } * core::mem::size_of::<u32>();
    let descriptor_config = descriptor::DescriptorConfig {
        // memory
        source: source,
        destination: ac_dac_txdata,
        byte_counter: byte_counter,
        // config
        link: None,
        wait_clock_cycles: 0,
        bmode: descriptor::BModeSel::Normal,
        // destination
        dest_width: descriptor::DataWidth::Bit32,
        dest_addr_mode: descriptor::AddressMode::IoMode,
        dest_block_size: descriptor::BlockSize::Byte4,
        dest_drq_type: descriptor::DestDrqType::AudioCodec,
        // source
        src_data_width: descriptor::DataWidth::Bit32,
        src_addr_mode: descriptor::AddressMode::LinearMode,
        src_block_size: descriptor::BlockSize::Byte4,
        src_drq_type: descriptor::SrcDrqType::Dram,
    };

    // create descriptor
    let mut descriptor: Descriptor = descriptor_config.try_into().unwrap();
    let descriptor_ptr = &descriptor as *const Descriptor as *const ();

    // link descriptor to itself
    descriptor.link = descriptor_ptr as u32;
    descriptor.link |= ((descriptor_ptr as usize >> 32) as u32) & 0b11;

    descriptor
}

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 4. Enable the DAC DRQ and DMA.
fn enable_dac_drq_dma(audio_codec: &AUDIO_CODEC, dmac: &mut Dmac, descriptor: &descriptor::Descriptor) {
    unsafe {
        let channel = 0;

        // enable interrupts
        dmac.irq_en0().write(|w| {
            w.dma0_hlaf_irq_en().set_bit()
             .dma0_pkg_irq_en().set_bit()
             .dma0_queue_irq_en().set_bit()
        });

        dmac.channels[channel]
            .set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Handshake);
        //    .set_channel_modes(dmac::ChannelMode::Handshake, dmac::ChannelMode::Handshake);
        //    .set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Wait);
        //    .set_channel_modes(dmac::ChannelMode::Handshake, dmac::ChannelMode::Wait);
        dmac.channels[channel].start_descriptor(NonNull::from(descriptor));
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
        w.txe_int().set_bit()
         .txu_int().set_bit()
         .txo_int().set_bit()
    });

    // enable dac interrupts - all of them here ?
    audio_codec.ac_dac_fifoc.modify(|_, w| {
        w.dac_drq_en().enable()
         .dac_irq_en().enable()
         .fifo_underrun_irq_en().enable()
         .fifo_overrun_irq_en().enable()
    });
}

// - Trim bandgap reference voltage. ------------------------------------------

// from: https://github.com/orangecms/oreboot/blob/sunxi/nezha/src/mainboard/sunxi/nezha/bt0/src/main.rs#L356

use core::ptr::{read_volatile, write_volatile};

pub fn trim_bandgap_ref_voltage() {
    let soc_version = (unsafe { read_volatile(SOC_VER_REG as *mut u32) >> 22 }) & 0x3f;
    println!("v {}", soc_version);

    let mut bg_trim = (unsafe { read_volatile(BANDGAP_TRIM_REG as *mut u32) } >> 16) & 0xff;
    if bg_trim == 0 {
        bg_trim = 0x19;
    }

    let reg = CCU_AUDIO_SMTH as u32;
    clrbits_le32(reg, GATING_BIT);
    udelay(2);
    clrbits_le32(reg, RST_BIT);
    udelay(2);
    /* deassert audio codec reset */
    setbits_le32(reg, RST_BIT);
    /* open the clock for audio codec */
    setbits_le32(reg, GATING_BIT);

    if soc_version == 0b1010 || soc_version == 0 {
        setbits_le32((SUNXI_AUDIO_CODEC + 0x31C) as u32, 1 << 1);
        setbits_le32((SUNXI_AUDIO_CODEC + 0x348) as u32, 1 << 30);
    }

    // TODO: recheck
    let val = unsafe { read_volatile(AC_SMTH as *mut u32) };
    unsafe { write_volatile(AC_SMTH as *mut u32, (val & 0xffffff00) | bg_trim) };
}

fn clrbits_le32(reg: u32, val: u32) {
    unsafe {
        let cval = read_volatile(reg as *mut u32);
        write_volatile(reg as *mut u32, cval & !val);
    }
}

fn setbits_le32(reg: u32, val: u32) {
    unsafe {
        let cval = read_volatile(reg as *mut u32);
        write_volatile(reg as *mut u32, cval | val);
    }
}

fn udelay(micros: usize) {
    unsafe {
        for _ in 0..micros {
            core::arch::asm!("nop")
        }
    }
}

pub const SUNXI_AUDIO_CODEC: u32 = 0x0203_0000;
const AC_SMTH: u32 = SUNXI_AUDIO_CODEC + 0x348;

const SUNXI_SID_BASE: u32 = 0x0300_6000;
const SOC_VER_REG: u32 = SUNXI_SID_BASE + 0x200;
const BANDGAP_TRIM_REG: u32 = SUNXI_SID_BASE + 0x228;

const CCU_BASE: usize = 0x0200_1000;
const CCU_AUDIO_SMTH: usize = CCU_BASE + 0x0a5c;

const GATING_BIT: u32 = 1 << 0;
const RST_BIT: u32 = 1 << 16;
