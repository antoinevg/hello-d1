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
///    AUDIO_CODEC_DAC_CLK_REG and PLL_AUDIO1_CTRL_REG to configure
///    PLL_Audio1 frequency and enable PLL_Audio1. For details, refer
///    to section 3.2 “CCU”.
/// 2. Configure the sample rate and data transfer format, then open
///    the DAC.
/// 3. Configure the DMA and DMA request.
/// 4. Enable the DAC DRQ and DMA.
///

pub(crate) mod registers;

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{AUDIOCODEC, CCU};

use crate::dmac::{self, descriptor, descriptor::DescriptorConfig, Dmac};
use twiddle;
use crate::println;

use registers::ac_dac_fifoc_ext;
use registers::ac_dac_fifoc_ext::Rext;
use registers::ac_dac_fifoc_ext::Wext;

/// Built-in audio codec
pub struct AudioCodec {
    audio_codec: AUDIOCODEC,
}

impl AudioCodec {
    /// Create a new `AudioCodec`
    ///
    /// TODO replace magic numbers with register methods
    pub fn new(audio_codec: AUDIOCODEC, ccu: &mut CCU, dmac: &mut Dmac) -> AudioCodec {
        init_codec_ccu(ccu); // 1
        configure_dac(&audio_codec); // 2
        // NOOP configure_dma(&audio_codec, dmac); // 3

        // debug mode
/*
        let bits = audio_codec.ac_dac_dg.read().bits();
        twiddle::set(bits, 11, true); // DAC_MODU_SELECT (0: Normal, 1: Debug)
        twiddle::set_range(bits, 9..10, 0b01); // DAC_PATTERN_SELECT (0b00: Normal, 0b01: -6 dB Sine wave)
        twiddle::set(bits, 8, true); // CODEC_CLK_SELECT (0: PLL, 1: OSC for Debug)
        twiddle::set(bits, 6, false); // DA_SWP  (0: Disabled, 1: Enabled)
        twiddle::set_range(bits, 0..2, 0b000); // ADDA_LOOP_MODE (0b000: Disabled, 0b001: ADC1/ADC2, 0b010: ADC3)
        audio_codec.ac_dac_dg.write(|w| unsafe { w.bits(bits) });
*/
        //init_codec_ccu(ccu); // 1
        //configure_dac(&audio_codec); // 2

        Self { audio_codec }
    }

    pub fn start(&self, dmac: &mut Dmac) {
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
            audio_codec: d1_pac::Peripherals::steal().AUDIOCODEC,
        }
    }
}

// ----------------------------------------------------------------------------

/// 8.4.4.2 Playback Process, page 769-770
///
/// 1. Codec initialization: configure AUDIO_CODEC_BGR_REG
///    to open the audio codec bus clock gating and de-assert bus
///    reset; configure AUDIO_CODEC_DAC_CLK_REG and PLL_AUDIO1_CTRL_REG
///    to configure PLL_Audio1 frequency and enable PLL_Audio1. For
///    details, refer to section 3.2 “CCU”.
/// ...
///
/// Also see: d1-sdk.repo/lichee/brandy-2.0/u-boot-2018/drivers/sound/sun8iw18-codec.c
///           sunxi_codec_init()
///
fn init_codec_ccu(ccu: &CCU) {
    // configure PLL_Audio0 frequency and enable PLL_Audio1
    // TODO Shouldn't this be PLL_Audio1 ??? (see pg. 769 Playback Process)
    //
    // pg. 64 3.2.6.7 PLL_AUDIO0_CTRL_REG
    // 0x0078 PLL_AUDIO0 Control Register (Default Value: 0x4814_5500)
    // Example: 1010_1001_0000_1011_0001_0111_0000_0001
    //
    // 31     PLL Enable               = 1
    // 30     LDO Enable               = 0
    // 29:28  LOCK Enable + PLL Lock   = 10
    // 27     PLL Output Gate Enable   = 1
    // 26:25  ---                      = 00
    // 24     PLL SDM Enable           = 1
    // 23:22  ---                      = 00
    // 21:16  PLL Post-div P           = 00_1011   = 11
    // 15:08  PLL N                    = 0001_0111 = 23
    // 07:06  PLL Unlock Level         = 00
    // 05     PLL Lock Level           = 0
    // 04:02  ---                      = 000
    // 01     PLL Input Div M1         = 0
    // 00     PLL Ouput Div M0         = 1
    ccu.pll_audio0_ctrl
        .write(|w| unsafe { w.bits(0xa90b_1701) });

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

    // open audio codec bus clock gating and de-assert bus reset
    //
    // pg. 119 3.2.6.86 AUDIO_CODEC_BGR_REG
    // 0x0A5C AUDIO_CODEC Bus Gating Reset Register (Default Value: 0x0000_0000)
    //
    // 31:17  ---
    // 16     AUDIO_CODEC Reset            = 1 (0: Assert, 1: De-assert)
    // 15:01  ---
    // 00     Gating Clock for AUDIO_CODEC = 0 (0: Mask, 1: Pass)
    ccu.audio_codec_bgr
        .write(|w| unsafe { w.bits(0x0001_0000) });
    ccu.audio_codec_bgr
        .write(|w| unsafe { w.bits(0x0001_0001) });

    // configure PLL_Audio0 frequency and enable PLL_Audio1
    //
    // pg. 117 3.2.6.84 AUDIO_CODEC_DAC_CLK_REG
    // 0x0A50 AUDIO_CODEC_DAC Clock Register (Default Value: 0x0000_0000)
    // Example: 1000_0000_0000_0000_0000_0000_0000_0000
    //
    // 31     Gating Clock        = 1     (0: off, 1: on)
    // 30:27  ---
    // 26:24  Clock Source Select = 000   (00: PLL_AUDIO0(1x), 01: PLL_AUDIO1(DIV2), 10: PLL_AUDIO1(DIV5))
    // 23:10  ---
    // 09:08  Factor N            = 00    (00: /1, 01: /2, 10: /4, 11: /8)
    // 07:05  ---
    // 04:00  Factor M            = 00000 (M = FACTOR_M + 1)
    ccu.audio_codec_dac_clk
        .write(|w| unsafe { w.bits(0x8000_0000) });

    // TODO
    // pg 118 3.2.6.85
    // 0x0A54 AUDIO_CODEC_ADC Clock Register (Default Value: 0x0000_0000)



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
fn configure_dac(codec: &AUDIOCODEC) {
    use registers::ac_dac_fifoc_ext::{self, *};

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
    // 03     DAC_IRQ_EN             = 0
    // 02     FIFO_UNDERRUN_IRQ_EN   = 0
    // 01     FIFO_OVERRUN_IRQ_EN    = 9
    // 00     FIFO_FLUSH             = 0   (0: self clear, 1: flush TX FIFO)

    // - sunxi_codec_hw_params() --

    let bits = codec.ac_dac_fifoc.read().bits();

    // set DAC sample resolution
    let bits = twiddle::set_range(bits, 24..25, 0b01); // FIFO Mode = [15:0] (little-endian)
    let bits = twiddle::set(bits, 5, false); // Sample Resolution = 16 bits

    // set DAC sample rate
    //codec.ac_dac_fifoc.modify(|_r, w| w.sample_rate(SampleRate::R48K));
    //codec.ac_dac_fifoc.modify(|_r, w| w.sample_rate().r48k());
    //let is_r48k = codec.ac_dac_fifoc.read().sample_rate().is_r48k();
    let bits = twiddle::set_range(bits, 29..31, SampleRate::R48k.into());

    // set DAC channels
    let bits = twiddle::set(bits, 6, false); // DAC Mono Enable = false

    // Write to register
    codec.ac_dac_fifoc.write(|w| unsafe { w.bits(bits) });

    // - sunxi_codec_playback_prepare --

    // FIFO flush
    let bits = codec.ac_dac_fifoc.read().bits();
    let bits = twiddle::set(bits, 0, true);
    codec.ac_dac_fifoc.write(|w| unsafe { w.bits(bits) });

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
    codec.ac_dac_fifos.write(|w| unsafe { w.bits(0b1110) });

    // FIFO clear sample counter
    //
    // pg. 780 8.4.6. AC_DAC_CNT6
    // 0x0024 DAC TX Counter Register (Default Value: 0x0000_0000)
    //
    // 31:00  TX_CNT TX Sample Counter
    codec.ac_dac_cnt.write(|w| unsafe { w.bits(0b1110) });
    let bits = codec.ac_dac_fifoc.read().bits();
    let bits = twiddle::set(bits, 0, true);
    codec.ac_dac_fifoc.write(|w| unsafe { w.bits(bits) });

    // FIFO enable empty DRQ
    let bits = codec.ac_dac_fifoc.read().bits();
    let bits = twiddle::set(bits, 4, true);
    codec.ac_dac_fifoc.write(|w| unsafe { w.bits(bits) });
/*
    // Enable ADC analog channels
    //
    // pg. 823 8.4.6.112 ADC1 Analog Control Register (Default Value: 0x001C_C055)
    // 0x0300 ADC1_REG
    //
    // 31 ADC1_EN
    // ...
    //
    // pg. 826 8.4.6.113 ADC2 Analog Control Register (Default Value: 0x001C_0055
    // 0x0304 ADC2_REG
    //
    // 31 ADC2_EN
    // ...
    let adc1_reg = codec.adc1_reg.read().bits();
    let adc1_reg = twiddle::set(adc1_reg, 31, true);
    codec.adc1_reg.write(|w| unsafe { w.bits(adc1_reg) });
    let adc2_reg = codec.adc2_reg.read().bits();
    let adc2_reg = twiddle::set(adc2_reg, 31, true);
    codec.adc2_reg.write(|w| unsafe { w.bits(adc2_reg) });
*/
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
    let bits = codec.dac_reg.read().bits();
    let bits = twiddle::set_range(bits, 10..15, 0b111111);
    let bits = twiddle::set_range(bits, 0..4, 0x1f);
    codec.dac_reg.write(|w| unsafe { w.bits(bits) });

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
    const AC_HP2_REG: *mut u32 = (SUNXI_AUDIO_CODEC + 0x340) as *mut u32;
    let bits = unsafe { core::ptr::read_volatile(AC_HP2_REG) };
    let bits = twiddle::set_range(bits, 28..30, 0b000);
    let bits = twiddle::set_range(bits, 24..25, 0b01);
    let bits = twiddle::set(bits, 21, true);
    let bits = twiddle::set(bits, 20, true);
    unsafe { core::ptr::write_volatile(AC_HP2_REG, bits) };

    // Set DAC Volume
    //
    // pg. 775 8.4.6.2 DAC Volume Control Register (Default Value: 0x0000_A0A0)
    // 0x0004 DAC_VOL_CTRL
    //
    // 31:17  ---
    // 16     DAC_VOL_SEL - DAC Volume Control Selection Enable
    // 15:08  DAC_VOL_L  (0x00: Mute, 0x01: -119.25dB 0xa0: 0dB, 0xff: 71.25dB, 0.75dB / Step)
    // 07:00  DAC_VOL_R
    let bits = codec.dac_vol_ctrl.read().bits();
    let bits = twiddle::set(bits, 17, true);
    let bits = twiddle::set_range(bits, 8..15, 0xa0);
    let bits = twiddle::set_range(bits, 0..7, 0xa0);
    codec.dac_vol_ctrl.write(|w| unsafe { w.bits(bits) });

    // Enable DAC digital part
    //
    // pg. 774 8.4.6.1 DAC Digital Part Control Register (Default Value: 0x0000_0000)
    // 0x0000 AC_DAC_DPC
    //
    // 31  EN_DAC  DAC Digital Part Enable
    // ...
    let bits = codec.ac_dac_dpc.read().bits();
    let bits = twiddle::set(bits, 31, true);
    codec.ac_dac_dpc.write(|w| unsafe { w.bits(bits) });

    // Turn on speaker ???
    unsafe {
        riscv::asm::delay(1000);
    } // increase in case of pop
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

pub const BLOCK_LENGTH: usize = 128;
pub const STEREO_BLOCK_LENGTH: usize = BLOCK_LENGTH * 2;    // 2 channels
pub const TX_BUFFER_LENGTH:usize = STEREO_BLOCK_LENGTH * 2; // 2 half-blocks
pub static mut TX_BUFFER: [u32; TX_BUFFER_LENGTH] = [0; TX_BUFFER_LENGTH];

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 3. Configure the DMA and DMA request.
/// ...
fn configure_dma(codec: &AUDIOCODEC, dmac: &mut Dmac) {
    // TODO move dma config here
}

/// 8.4.4.2 Playback Process, page 769-770
///
/// ...
/// 4. Enable the DAC DRQ and DMA.
fn enable_dac_drq_dma(codec: &AUDIOCODEC, dmac: &mut Dmac) {
    // generate some test noise
    unsafe {
        let mut counter = 0;
        for byte in &mut TX_BUFFER {
            *byte = ((u32::MAX / TX_BUFFER.len() as u32)  * counter) as u32;
            counter += 1;
        }
    }

    // enable dac drq
    let bits = codec.ac_dac_fifoc.read().bits();
    let bits = twiddle::set(bits, 4, true); // DAC_DRQ_EN
    let bits = twiddle::set(bits, 3, true); // DAC_IRQ_EN
    codec.ac_dac_fifoc.write(|w| unsafe { w.bits(bits) });

    // enable dma
    let ac_dac_txdata = &unsafe { &*AUDIOCODEC::PTR }.ac_dac_txdata as *const _ as *mut ();
    let descriptor_config = descriptor::DescriptorConfig {
        source: unsafe { TX_BUFFER.as_ptr().cast() },
        destination: ac_dac_txdata,
        byte_counter: unsafe { TX_BUFFER.len() },
        link: None,
        wait_clock_cycles: 0,
        bmode: descriptor::BModeSel::Normal,
        dest_width: descriptor::DataWidth::Bit32,
        dest_addr_mode: descriptor::AddressMode::IoMode,
        dest_block_size: descriptor::BlockSize::Byte4,
        dest_drq_type: descriptor::DestDrqType::AudioCodec,
        src_data_width: descriptor::DataWidth::Bit32,
        src_addr_mode: descriptor::AddressMode::LinearMode,
        src_block_size: descriptor::BlockSize::Byte1,
        src_drq_type: descriptor::SrcDrqType::Dram,
    };
    let descriptor = descriptor_config.try_into().unwrap();

    unsafe {
        dmac.channels[2].set_channel_modes(dmac::ChannelMode::Wait, dmac::ChannelMode::Handshake);
        dmac.channels[2].start_descriptor(NonNull::from(&descriptor));
    }
}

// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------
// - --------------------------------------------------------------------------


/*
/// Configure Mixer
///
/// Also see: d1-sdk.repo/lichee/brandy-2.0/u-boot-2018/drivers/sound/sun8iw18-codec.c
///           sunxi_codec_init()
pub(crate) fn configure_mixer(codec: &AUDIOCODEC) {
    // set dac digital volume (0*-1.16=0dB)
    codec
        .ac_dac_dpc
        .modify(|r, w| unsafe { w.bits(r.bits() | 0x0) }); // TODO check

    // disable mic2, mic3 boost amplifier
    let mic2_mic3_ctl = 0; // SUNXI_MIC2_MIC3_CTL: 0x308 + 0 = 0x308;
    codec
        .adc3_reg
        .write(|w| unsafe { w.bits(0x44 << mic2_mic3_ctl) });
    // OR
    /*unsafe {
        let ptr = codec.adc3_reg.as_ptr();
        twiddle::reg_set(ptr, 0, false);
    }*/

    // mute ladc mixer
    let ladcmix_src = 1; // SUNXI_LADCMIX_SRC: 0x308 + 1 = 0x309;
    codec
        .adc3_reg
        .modify(|r, w| unsafe { w.bits(r.bits() | (0x00 << ladcmix_src)) }); // TODO check

    // set lineout volume:  -(31-0x19)*1.5 = -9dB
    let lineout_ctl1 = 2; // SUNXI_LINEOUT_CTL1: 0x304 + 2  = 0x306
    codec
        .adc2_reg
        .modify(|r, w| unsafe { w.bits(r.bits() | (0x19 << lineout_ctl1)) }); // TODO check

    // set right lineout source
    let lineout_ctl0 = 1; // SUNXI_LINEOUT_CTL0: 0x304 + 1  = 0x305
    codec
        .adc2_reg
        .modify(|r, w| unsafe { w.bits(r.bits() | (0x01 << lineout_ctl0)) }); // TODO check
}


pub(crate) fn prepare_playback(p: &pac::Peripherals) {

    // - sunxi_codec_playback_prepare -----------------------------------------

    // flush fifo

    // clear pending

    // clear sample counter

    // enable fifo empty drq

    // enable dac analog left channel

    // enable dac digital part

    // enable left, right lineout
}

pub(crate) fn init_dma(p: &pac::Peripherals) {
    // - sunxi_dma_start ------------------------------------------------------

    // configure dma
}

pub(crate) fn start_audio(p: &pac::Peripherals) {

}

// - interrupts ---------------------------------------------------------------

//use d1_pac::interrupt::Interrupt::AUDIO_CODEC;

//#[export_name = "AUDIO_CODEC"]
//fn AUDIO_CODEC() {
//}

//use d1_pac::Interrupt::AUDIO_CODEC;

*/

// - example code -------------------------------------------------------------
//
// https://github.com/Tina-Linux/tina-v83x-u-boot-2018/blob/master/drivers/sound/sun8iw18-codec.c
// https://github.com/Tina-Linux/tina-v83x-linux-4.9/blob/master/sound/soc/sunxi/sun50iw3-codec.c
//
// https://github.com/nikicoon/src/blob/trunk/sys/arch/arm/sunxi/sunxi_codec.c
// https://github.com/cws06/fstack/blob/master/freebsd/arm/allwinner/a10_codec.c
// https://github-com.translate.goog/PeishengYE/A13_boot_code/blob/master/boot1/apps/Card_Android/Common/codec_led/codec_led.c?_x_tr_sl=auto&_x_tr_tl=en-US&_x_tr_hl=en-US

// # Rust sample code
//
// https://github.com/barafael/rust-sipeed-longan-DAC/



// https://github.com/orangecms/oreboot/blob/sunxi/nezha/src/mainboard/sunxi/nezha/bt0/src/main.rs#L387
// https://matrix.to/#/!iDVTgmAFCmiJLtdsQo:matrix.org/$3imYpn1DOGAh22s8ppYEgMDmobl4R7r5RsiNvSvFIk0
use core::ptr::{read_volatile, write_volatile};

/* Trim bandgap reference voltage. */
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
