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

use d1_pac as pac;


pub(crate) fn init_codec(p: &pac::Peripherals) {

    // - sunxi_codec_init - ccu -----------------------------------------------
    let ccu = &p.CCU;
    ccu.pll_audio0_ctrl.write(|w|      unsafe { w.bits(0xa90b_1701) });
    ccu.pll_audio0_pat0_ctrl.write(|w| unsafe { w.bits(0xc001_26e9) });
    ccu.pll_audio0_pat1_ctrl.write(|w| unsafe { w.bits(0x0000_0000) });
    ccu.pll_audio0_bias.write(|w|      unsafe { w.bits(0x0003_0000) });
    ccu.audio_codec_bgr.write(|w|      unsafe { w.bits(0x0001_0000) });
    ccu.audio_codec_bgr.write(|w|      unsafe { w.bits(0x0001_0001) });
    ccu.audio_codec_dac_clk.write(|w|  unsafe { w.bits(0x8000_0000) });

    // - sunxi_codec_init - audio_codec ---------------------------------------
    let codec = &p.AUDIOCODEC;

    // set dac digital volume (0*-1.16=0dB)
    codec.ac_dac_dpc.modify(|r, w| unsafe { w.bits(r.bits() | 0x0) });                    // TODO check

    // disable mic2, mic3 boost amplifier
    let mic2_mic3_ctl = 0;  // SUNXI_MIC2_MIC3_CTL: 0x308 + 0 = 0x308;
    codec.adc3_reg.write(|w|       unsafe { w.bits(0x44 << mic2_mic3_ctl) });

    // mute ladc mixer
    let ladcmix_src = 1;    // SUNXI_LADCMIX_SRC: 0x308 + 1 = 0x309;
    codec.adc3_reg.modify(|r, w|   unsafe { w.bits(r.bits() | (0x00 << ladcmix_src)) });  // TODO check

    // set lineout volume:  -(31-0x19)*1.5 = -9dB
    let lineout_ctl1 = 2;   // SUNXI_LINEOUT_CTL1: 0x304 + 2  = 0x306
    codec.adc2_reg.modify(|r, w|   unsafe { w.bits(r.bits() | (0x19 << lineout_ctl1)) }); // TODO check

    // set right lineout source
    let lineout_ctl0 = 1;   // SUNXI_LINEOUT_CTL0: 0x304 + 1  = 0x305
    codec.adc2_reg.modify(|r, w|   unsafe { w.bits(r.bits() | (0x01 << lineout_ctl0)) }); // TODO check
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

    // enable dac drq and dma

}

// - interrupts ---------------------------------------------------------------

//use d1_pac::interrupt::Interrupt::AUDIO_CODEC;

#[export_name = "AUDIO_CODEC"]
fn AUDIO_CODEC() {
}

//use d1_pac::Interrupt::AUDIO_CODEC;




// - example code -------------------------------------------------------------
//
// https://github.com/Tina-Linux/tina-v83x-u-boot-2018/blob/master/drivers/sound/sun8iw18-codec.c
// https://github.com/Tina-Linux/tina-v83x-linux-4.9/blob/master/sound/soc/sunxi/sun50iw3-codec.c
//
// https://github.com/nikicoon/src/blob/trunk/sys/arch/arm/sunxi/sunxi_codec.c
// https://github.com/cws06/fstack/blob/master/freebsd/arm/allwinner/a10_codec.c
// https://github-com.translate.goog/PeishengYE/A13_boot_code/blob/master/boot1/apps/Card_Android/Common/codec_led/codec_led.c?_x_tr_sl=auto&_x_tr_tl=en-US&_x_tr_hl=en-US
