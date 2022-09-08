/// 8.4 Audio Codec, page 758
///
/// One 128x20-bit FIFO for DAC data transmit
/// One 128x20-bit FIFO for ADC data receive
/// Programmable FIFO thresholds
/// HPF Function
///
/// The clock source for the digital part is the PLL_AUDIO0 and
/// PLL_AUDIO1.
///
/// For the ADC clock, configure AUDIO_CODEC_ADC_CLK_REG[25:24] to
/// select the clock source.
///
/// For the DAC clock, configure AUDIO_CODEC_DAC_CLK_REG[25:24] to
/// select the clock source.
///
/// The PK-PK jitter of PLL_AUDIO0 and PLL_AUDIO1 should be less than
/// 200 ps.
///
/// The digital ADC part can be enabled or disabled by the bit[28] of the AC_ADC_FIFOC register.
///
/// The stereo DAC sample rate can be configured by setting the
/// register. To save power, the analog DACL can be enabled or
/// disabled by setting the bit[15] of the DAC_REG register, and the
/// analog DACR can be enabled or disabled by setting the bit[14] of
/// the DAC_REG register. The digital DAC part can be enabled or
/// disabled by the bit[31] of the AC_DAC_DPC register.
///
/// The Audio Codec has two interrupts:
/// ADC_IRQ_EN
/// ADC_IRQ_STATUS
/// ADC_OVERRUN_IRQ_EN
/// ADC_OVERRUN_IRQ_STATUS
/// DAC_IRQ_EN
/// DAC_IRQ_STATUS
/// DAC_OVERRUN_IRQ_EN
/// DAC_OVERRUN_IRQ_STATUS
/// DAC_UNDERRUN_IRQ_EN
/// DAC_UNDERRUN_IRQ_STATUS
///
/// 8.4.4.1 Record Process, page 769
/// 1. Codec initialization: configure AUDIO_CODEC_BGR_REG to open the
///    audio codec bus clock gating and de-assert bus reset; configure
///    AUDIO_CODEC_ADC_CLK_REG and PLL_AUDIO0_CTRL_REG to configure
///    PLL_Audio0 frequency and enable PLL_Audio0. For details, refer
///    to section 3.2 “CCU”.
/// 2. Configure the sample rate and data transfer format, then open
///    the ADC.
/// 3. Configure the DMA and DMA request.
/// 4. Enable the ADC DRQ and DMA.
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
    ///
    /// TODO replace magic numbers with register methods
    pub fn new(audio_codec: AUDIO_CODEC, ccu: &mut CCU) -> AudioCodec {
        init_codec_ccu(ccu); // 1
        configure_dac(&audio_codec); // 2

        // enable_debug_mode(&audio_codec);

        Self { audio_codec }
    }

    pub fn start(&self, dmac: &mut Dmac) {
        configure_mixer(&self.audio_codec);
        enable_dac_drq_dma(&self.audio_codec, dmac); // 4
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
         .codec_clk_select().osc()     // CODEC_CLK_SELECT (0: PLL, 1: OSC for Debug)
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
    ccu.audio_codec_bgr.write(|w| w.gating().pass());
    ccu.audio_codec_bgr.write(|w| w.rst().deassert());

    // 2. configure PLL_Audio0 frequency and select PLL_Audio0
    //
    // pg. 117 3.2.6.84 AUDIO_CODEC_DAC_CLK_REG
    // 0x0A50 AUDIO_CODEC_DAC Clock Register (Default Value: 0x0000_0000)
    // Example: 1000_0000_0000_0000_0000_0000_0000_0000
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

    // 3. configure PLL_Audio0 frequency and enable PLL_Audio0
    //
    // pg. 64 3.2.6.7 PLL_AUDIO0_CTRL_REG
    // 0x0078 PLL_AUDIO0 Control Register (Default Value: 0x4814_5500)
    // Default: 0100_1000_0001_1000_0101_0101_0000_0000
    // Example: 1010_1001_0000_1011_0001_0111_0000_0001
    //
    // PLL_AUDIO0(1X) = (24MHz*N/M1/M0)/P/4 = (24000000 * 23 / 1 / 2) / 11 / 4 = 6272727.2727272725 ????
    //
    // 31     PLL Enable               = 1
    // 30     LDO Enable               = 0
    // 29:28  LOCK Enable + PLL Lock   = 10
    // 27     PLL Output Gate Enable   = 1
    // 26:25  ---
    // 24     PLL SDM Enable           = 1         = Enable spread spectrum and decimal division
    // 23:22  ---
    // 21:16  PLL Post-div P           = 00_1011   = 11
    // 15:08  PLL N                    = 0001_0111 = 23
    // 07:06  PLL Unlock Level         = 00        = 21-29 Clock Cycles
    // 05     PLL Lock Level           = 0         = 24-26 Clock Cycles
    // 04:02  ---
    // 01     PLL_INPUT_DIV_2          = 0         = M1 = PLL_INPUT_DIV2  + 1 = 1
    // 00     PLL_OUTPUT_DIV_2         = 1         = M0 = PLL_OUTPUT_DIV2 + 1 = 2
    ccu.pll_audio0_ctrl.write(|w| unsafe {
        w.pll_ldo_en().disable()
         .lock_enable().enable()
         .pll_output_gate().enable()
         .lock_enable().clear_bit()
         .pll_sdm_en().disable() // ??? enable spread spectrum?
         .pll_p().bits(11)
         .pll_n().bits(23)
         .pll_unlock_mdsel().cc_21_29()
         .pll_lock_mdsel().cc_24_26()
         .pll_input_div2().clear_bit()
         .pll_output_div2().set_bit()
    });
    ccu.pll_audio0_ctrl
        .write(|w| unsafe { w.pll_en().enable() });

    while ccu.pll_audio0_ctrl.read().lock_enable().is_disable() {
        println!("waiting for pll_audio0 lock");
    }

    /*
        // pg. 74 3.2.6.19 PLL_AUDIO0_PAT0_CTRL_REG
        // 0x0178 PLL_AUDIO0 Pattern0 Control Register (Default Value: 0x0000_0000)
        // Example: 1100_0000_0000_0001_0010_0110_1110_1001
        //
        // 31     Sigma-Delta Pattern Enable = 1
        // 30:29  Spread Frequency Mode      = 10 = Triangular (1-bit)
        // 28:20  Wave Step                  = 0_0000_0000
        // 19     SDM Clock Select           = 0  = 24 MHz
        // 18:17  Frequency                  = 00 = 31.5 kHz
        // 16:00  Wave Bottom                = 1_0010_0110_1110_1001 = 0x1_26e9 = 75497
        ccu.pll_audio0_pat0_ctrl
            .write(|w| unsafe { w.bits(0xc001_26e9) });

        // pg. 75 3.2.6.20 PLL_AUDIO0_PAT1_CTRL_REG
        // 0x017C PLL_AUDIO0 Pattern1 Control Register (Default Value: 0x0000_0000)
        //
        // 31:25  ---
        // 24     Dither Enable
        // 23:21  ---
        // 20     Fraction Enable
        // 19:17  ---
        // 16:00  Fraction In
        ccu.pll_audio0_pat1_ctrl
            .write(|w| unsafe { w.bits(0x0000_0000) });

        // pg. 78 3.2.6.29 0x0378 PLL_AUDIO0_BIAS_REG
        // PLL_AUDIO0 Bias Register (Default Value: 0x0003_0000)
        //
        // 31:21  ---
        // 20:16  PLL bias control = 3
        // 15:00  ---
        ccu.pll_audio0_bias
            .write(|w| unsafe { w.bits(0x0003_0000) });

    */
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
    audio_codec.ac_dac_fifoc.write(|w| unsafe {
        w.dac_fs().fs48k()
         .send_last().zero()
         .fifo_mode().little_endian()
         .dac_drq_clr_cnt().wlevel() // ?
         .tx_trig_level().bits(32) // ?
         .dac_mono_en().stereo()
         .tx_sample_bits().bits_16()
    });

    // FIFO flush
    //audio_codec.ac_dac_fifoc.write(|w| w.fifo_flush().flush());

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
    /*audio_codec.ac_dac_fifos.write(|w| {
        w.txe_int().clear_or_pending()
         .txu_int().clear_or_pending()
         .txo_int().clear_or_pending()
    });*/

    // enable FIFO_UNDERRUN & FIFO_OVERRUN IRQ's
    /*audio_codec.ac_dac_fifoc.write(|w| {
        w.fifo_underrun_irq_en().enable()
         .fifo_overrun_irq_en().enable()
    });*/

    // FIFO clear sample counter
    //
    // pg. 780 8.4.6. AC_DAC_CNT
    // 0x0024 DAC TX Counter Register (Default Value: 0x0000_0000)
    //
    // 31:00  TX_CNT TX Sample Counter
    audio_codec.ac_dac_cnt.write(|w| unsafe { w.bits(0) });

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
    audio_codec.ac_dac_dpc.write(|w| unsafe {
        w.en_da().enable().dvol().bits(0) // ATT = 0 * -1.16dB = 0dB
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
    audio_codec.dac_vol_ctrl.write(|w| unsafe {
        w.dac_vol_sel().enable()
         .dac_vol_l().bits(0xa0)
         .dac_vol_r().bits(0xa0)
    });
}

fn configure_mixer(audio_codec: &AUDIO_CODEC) {
    // Enable left and right line out
    //
    // pg. 832 8.4.6.115 DAC Analog Control Register (Default Value: 0x0015_0000)
    // 0x0310 DAC_REG
    //
    // 31:24  ---
    // ...
    // 15     DACL_EN ???
    // 14     DACR_EN ???
    // 13     LINEOUTLEN
    // 12     LMUTE            (0: Mute, 1: Unmute)
    // 11     LINEOUTREN
    // 10     RMUTE            (0: Mute, 1: Unmute)
    // 09:07  ---
    // 06     LINEOUTL_DIFFEN
    // 05     LINEOUTR_DIFFEN
    // 04:00  LINEOUT_VOL_CTRL (0x1f to 0x02, step=-1.5dB, mute = 0 or 1)
    audio_codec.dac.write(|w| unsafe {
        w.dacl_en().enable()
         .dacr_en().enable()
         .lineoutlen().enable()
         .lmute().unmute()
         .lineoutren().enable()
         .rmute().unmute()
         .lineout_vol_ctrl().bits(0x1f)
    });

    // Enable headphone output
    //
    // pg. 844 8.4.6.121 Headphone2 Analog Control Register (Default Value: 0x0640_4000)
    // 0x0340 HP2_REG
    //
    // 31     HPFB_BUF_EN - Headphone feedback buffer op enable
    // 30:28  HEADPHONE GAIN  (0b000: 0dB, 0b001: -6dB ... 0b111: -42dB, -6dB step)
    // 27:26  HPFB_RES
    // 25:24  OPDRV_CUR       (00: Min, 01: Max)
    // 23:22  IOPHP
    // 21     HP_DRVEN - Headphone Driver enable
    // 20     HP_DRVOUTEN - Headphone Driver Output Enable
    // 19     RSWITCH
    // 18     RAMPEN
    // 17     HPFB_IN_EN
    // 16     RAMP_FINAL_CONTROL
    // 15     RAMP_OUT_EN
    // 14:13  RAMP_FINAL_STATE_RES
    // 09:08  HPFB_BUF_OUTPUT_CURRENT
    // 07:00  ---
    audio_codec.hp2.write(|w| {
        w.hpfb_buf_en().enable() // ?
         .headphone_gain().db0()
         .opdrv_cur().max()
         .hp_drven().enable()
         .hp_drvouten().enable()
    });

    // Turn on speaker ???
    //unsafe {
    //    riscv::asm::delay(1000);
    //} // increase in case of pop
    // looks like this is actually NC on the RV Dock
    // let gpio = unsafe { pac::Peripherals::steal().GPIO };
    // const PB12: u32 = 12;
    // gpio.pb_cfg1.write(|w| w.pb12_select().input());
    // let bits = gpio.pb_dat.read().bits();
    // println!("GPIOB: {:#018b}", bits);    // 0b0001_1100_0000_0000
    // gpio.pb_cfg1.write(|w| w.pb12_select().output());
    // gpio.pb_dat
    //     .modify(|r, w| unsafe { w.bits(r.bits() | (0 << PB12)) });
}

pub const BLOCK_LENGTH: usize = 64;
pub const STEREO_BLOCK_LENGTH: usize = BLOCK_LENGTH * 2; // 2 channels
                                                         //pub const TX_BUFFER_LENGTH:usize = STEREO_BLOCK_LENGTH * 2; // 2 half-blocks
pub const TX_BUFFER_LENGTH: usize = STEREO_BLOCK_LENGTH * 1; // 2 buffers
pub static mut TX_BUFFER_1: [u32; TX_BUFFER_LENGTH] = [0; TX_BUFFER_LENGTH];
pub static mut TX_BUFFER_2: [u32; TX_BUFFER_LENGTH] = [0; TX_BUFFER_LENGTH];

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 3. Configure the DMA and DMA request.
/// ...
fn configure_dma(audio_codec: &AUDIO_CODEC, dmac: &mut Dmac) {
    // TODO move dma config here
}

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 4. Enable the DAC DRQ and DMA.
fn enable_dac_drq_dma(audio_codec: &AUDIO_CODEC, dmac: &mut Dmac) {
    // generate some test noise
    unsafe {
        let mut counter = 0;
        for frame in &mut TX_BUFFER_1 {
            *frame = ((u32::pow(2, 20) / TX_BUFFER_1.len() as u32) * counter) as u32;
            counter += 1;
        }
    }
    unsafe {
        let mut counter = 0;
        for frame in &mut TX_BUFFER_2 {
            *frame = ((u32::pow(2, 20) / TX_BUFFER_2.len() as u32) * counter) as u32;
            counter += 1;
        }
    }

    // TODO pg. 225  make sure that TX_BUFFER_1 is word-aligned

    // enable dma
    let ac_dac_txdata = &unsafe { &*AUDIO_CODEC::PTR }.ac_dac_txdata as *const _ as *mut ();
    let descriptor_config = descriptor::DescriptorConfig {
        // memory
        source: unsafe { TX_BUFFER_1.as_ptr().cast() },
        destination: ac_dac_txdata,
        //byte_counter: unsafe { TX_BUFFER_1.len() },     // ???
        byte_counter: (unsafe { TX_BUFFER_LENGTH } * 4), // ???
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
    let mut descriptor_config_2 = descriptor_config.clone();

    let mut descriptor_1: Descriptor = descriptor_config.try_into().unwrap();

    // link descriptor_1 to descriptor_2
    let descriptor_1_ptr = &descriptor_1 as *const Descriptor as *const ();
    descriptor_config_2.source = unsafe { TX_BUFFER_2.as_ptr().cast() };
    descriptor_config_2.link = Some(descriptor_1_ptr);
    let descriptor_2: Descriptor = descriptor_config_2.try_into().unwrap();

    // link descriptor_2 to descriptor_1
    let descriptor_2_ptr = &descriptor_2 as *const Descriptor as *const ();
    descriptor_1.link = descriptor_2_ptr as u32;
    descriptor_1.link |= ((descriptor_2_ptr as usize >> 32) as u32) & 0b11;

    unsafe {
        let channel = 2;

        // enable interrupts
        dmac.irq_en0().write(|w| {
            w //.dma2_hlaf_irq_en().set_bit()
                .dma2_pkg_irq_en()
                .set_bit()
                .dma2_queue_irq_en()
                .set_bit()
        });

        dmac.channels[channel]
            .set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Handshake);
        //dmac.channels[channel].set_channel_modes(dmac::ChannelMode::Handshake, dmac::ChannelMode::Handshake);
        //dmac.channels[channel].set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Wait);
        //dmac.channels[channel].set_channel_modes(dmac::ChannelMode::Handshake, dmac::ChannelMode::Wait);
        dmac.channels[channel].start_descriptor(NonNull::from(&descriptor_1));
    }

    // clear pending interrupts
    /*audio_codec.ac_dac_fifos.write(|w| {
        w.txe_int().clear_or_pending()
         .txu_int().clear_or_pending()
         .txo_int().clear_or_pending()
    });*/

    // enable dac interrupts - all of them here ?
    audio_codec.ac_dac_fifoc.write(|w| {
        w.dac_drq_en().enable()
         .dac_irq_en().enable()
        //.fifo_underrun_irq_en().enable()
        //.fifo_overrun_irq_en().enable()
    });

    // clear pending interrupts
    /*audio_codec.ac_dac_fifos.write(|w| {
        w.txe_int().clear_or_pending()
         .txu_int().clear_or_pending()
         .txo_int().clear_or_pending()
    });*/

    // FIFO flush
    //audio_codec.ac_dac_fifoc.write(|w| w.fifo_flush().flush());
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
