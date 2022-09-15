#![allow(dead_code, non_snake_case, unused_variables)]
#![no_std]
#![no_main]

mod dsp;
use dsp::osc;

use d1_pac as pac;
use panic_halt as _;

use hello_d1::audio_codec;
use hello_d1::dmac;
use hello_d1::logging;
use hello_d1::plic;
use hello_d1::println;

// Audio Buffer: 2 buffers * 2 channels * 32 samples = ~ 0.67 ms latency
static mut TX_BUFFER: [u32; 2 * 2 * 32] = [0; 2 * 2 * 32];

#[riscv_rt::entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    // logging
    logging::init(&p);
    println!("🦀 Hello Lichee RV D1!");

    // gpio - led
    let gpio = &p.GPIO;
    const PC1: u32 = 1;
    gpio.pc_cfg0.write(|w| w.pc1_select().output());

    // ccu
    let mut ccu = p.CCU;

    // audio_codec
    let mut audio_codec = audio_codec::AudioCodec::new(p.AUDIO_CODEC, &mut ccu);

    // dmac
    let mut dmac = dmac::Dmac::new(p.DMAC, &mut ccu);

    // plic
    let plic = plic::Plic::new(p.PLIC);
    unsafe {
        plic.set_priority(pac::Interrupt::DMAC_NS, plic::Priority::P1);
        plic.set_priority(pac::Interrupt::AUDIO_CODEC, plic::Priority::P1);
        plic.unmask(pac::Interrupt::DMAC_NS);
        plic.unmask(pac::Interrupt::AUDIO_CODEC);
    }

    // enable interrupts
    unsafe {
        riscv::interrupt::enable();
        riscv::register::mie::set_mext();
    }

    // audio_codec - start
    audio_codec.start(&mut dmac, unsafe { &TX_BUFFER });

    // blinky
    loop {
        gpio.pc_dat
            .modify(|r, w| unsafe { w.bits(r.bits() & (0 << PC1)) });
        unsafe {
            riscv::asm::delay(100_000_000);
        }
        gpio.pc_dat
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << PC1)) });
        unsafe {
            riscv::asm::delay(100_000_000);
        }
    }
}

/// DMA half-transfer interrupt handler for audio_codec data
unsafe fn handle_dma_interrupt(half: bool) {
    // oscillator state
    static mut OSC: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    static mut F: f32 = 0.;

    let half_buffer_length = TX_BUFFER.len() / 2;
    let skip = if half {
        0
    } else {
        half_buffer_length
    };

    // fill buffer
    let mut frame_index = 0;
    while frame_index < half_buffer_length {

        // whoop! whoop!
        F += 0.01;
        if F >= 500. {
            F = 0.;
        }
        OSC.dx = (1. / 48_000.) * (220. + F);
        let sample = dsp::f32_to_u20(OSC.step());

        let x = frame_index + skip;
        TX_BUFFER[x + 0] = sample as u32;
        TX_BUFFER[x + 1] = sample as u32;

        frame_index += 2;
    }
}

#[export_name = "MachineExternal"]
extern "C" fn MachineExternal() {
    let plic = unsafe { plic::Plic::summon() };

    // claim interrupt
    let claim = plic.claim();

    match claim {
        pac::Interrupt::DMAC_NS => {
            let dmac = unsafe { &*pac::DMAC::PTR };

            // get pending interrupts
            let pending = dmac.dmac_irq_pend0.read();

            // clear all pending interrupts
            if pending.dma0_hlaf_irq_pend().is_pending() {
                dmac.dmac_irq_pend0
                    .write(|w| w.dma0_hlaf_irq_pend().set_bit());
                while dmac.dmac_irq_pend0.read().dma0_hlaf_irq_pend().bit_is_set() {}
                unsafe { handle_dma_interrupt(true) };
            }
            if pending.dma0_pkg_irq_pend().is_pending() {
                dmac.dmac_irq_pend0
                    .write(|w| w.dma0_pkg_irq_pend().set_bit());
                while dmac.dmac_irq_pend0.read().dma0_pkg_irq_pend().bit_is_set() {}
                unsafe { handle_dma_interrupt(false) };
            }
        }
        pac::Interrupt::AUDIO_CODEC => {
            let audio_codec = unsafe { &*pac::AUDIO_CODEC::PTR };

            // get pending interrupts
            let pending = audio_codec.ac_dac_fifos.read();

            // clear all pending interrupts
            if pending.txo_int().is_pending() {
                println!("overrun");
                audio_codec.ac_dac_fifos.write(|w| w.txo_int().set_bit());
                while audio_codec.ac_dac_fifos.read().txo_int().bit_is_set() {}
            }
            if pending.txu_int().is_pending() {
                println!("underrun");
                audio_codec.ac_dac_fifos.write(|w| w.txu_int().set_bit());
                while audio_codec.ac_dac_fifos.read().txu_int().bit_is_set() {}
            }
        }
        x => {
            println!("Unexpected claim: {:?}", x);
        }
    }

    // Release claim
    plic.complete(claim);
}
