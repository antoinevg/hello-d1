#![allow(dead_code, non_snake_case, unused_variables)]
#![no_std]
#![no_main]

mod dsp;
use dsp::osc;

use d1_pac as pac;
use panic_halt as _;

use hello_d1::audio_codec::{self, BLOCK_LENGTH};
use hello_d1::dmac;
use hello_d1::logging;
use hello_d1::plic;
use hello_d1::println;

#[riscv_rt::entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    // logging
    logging::init(&p);
    println!("🦀 Hello Lichee RV D1!");

    // trim bandgap reference voltage - breaks DMA ???
    //audio_codec::trim_bandgap_ref_voltage();

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
    audio_codec.start(&mut dmac);

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


unsafe fn handle_dma_interrupt(half: bool) {
    // state
    static mut OSC: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    OSC.dx = (1. / 48_000.) * 440.;

    let skip = if half {
        (0, audio_codec::TX_BUFFER_LENGTH / 2)
    } else {
        (audio_codec::TX_BUFFER_LENGTH / 2, 0)
    };

    // fill buffer
    let half_buffer_length = audio_codec::TX_BUFFER_LENGTH / 2;
    let mut frame_index = 0;
    while frame_index < half_buffer_length {
        let tx0 = frame_index + skip.0;
        let tx1 = tx0 + 1;

        let sample = f32_to_u16(OSC.step()) as u32;
        audio_codec::TX_BUFFER_1[tx0] = sample;
        audio_codec::TX_BUFFER_1[tx1] = sample;

        frame_index += 2;
    }
}


#[inline(always)]
pub fn f32_to_u16(x: f32) -> u16 {
    //return (int16_t) __SSAT((int32_t) (x * 32767.f), 16);
    let x = x * 32_767.;
    let x = if x > 32_767. {
        32_767.
    } else if x < -32_768. {
        -32_768.
    } else {
        x
    };
    (x as i16) as u16
}

#[inline(always)]
fn f32_to_u20(x: f32) -> u32 {
    let x = x * 524_287.;
    let x = if x > 524_287. {
        524_287.
    } else if x < -524_288. {
        -524_288.
    } else {
        x
    };
    (x as i32) as u32
}


#[export_name = "MachineExternal"]
extern "C" fn MachineExternal() {
    static mut COUNTER: usize = 0;

    let plic = unsafe { plic::Plic::summon() };

    // Claim interrupt
    let claim = plic.claim();

    match claim {
        pac::Interrupt::GPIOE_NS => {
            let gpio = unsafe { &*pac::GPIO::PTR };

            // clear pending interrupts
            gpio.pe_eint_status
                .modify(|_r, w| w.eint8_status().set_bit());
            while gpio.pe_eint_status.read().eint8_status().bit_is_set() {}
        }
        pac::Interrupt::DMAC_NS => {
            let dmac = unsafe { &*pac::DMAC::PTR };

            // get some stats
            let packages = dmac.dmac_pkg_num0.read().bits();
            let left = dmac.dmac_bcnt_left0.read().bits();

            // get pending interrupts
            let pending = dmac.dmac_irq_pend0.read();

            // clear all pending interrupts
            if pending.dma0_hlaf_irq_pend().is_pending() {
                // half package
                dmac.dmac_irq_pend0
                    .write(|w| w.dma0_hlaf_irq_pend().set_bit());
                while dmac.dmac_irq_pend0.read().dma0_hlaf_irq_pend().bit_is_set() {}
                unsafe { handle_dma_interrupt(false) };
            }
            if pending.dma0_pkg_irq_pend().is_pending() {
                // end of package
                dmac.dmac_irq_pend0
                    .write(|w| w.dma0_pkg_irq_pend().set_bit());
                while dmac.dmac_irq_pend0.read().dma0_pkg_irq_pend().bit_is_set() {}
                unsafe { handle_dma_interrupt(true) };
            }
            if pending.dma0_queue_irq_pend().is_pending() {
                // end of queue
                println!("eoq");
                dmac.dmac_irq_pend0
                    .write(|w| w.dma0_queue_irq_pend().set_bit());
                while dmac.dmac_irq_pend0.read().dma0_queue_irq_pend().bit_is_set() {}
            }

            // dump some debug info
            let counter = unsafe { COUNTER };
            if counter % 1000 == 0 {
                println!("DMAC_NS pend:{:#05b}", pending.bits()); // 0: idle, 1: busy
                println!("DMAC_NS pkg:{}", packages);
                println!("DMAC_NS left:{}\n", left);
            }
        }
        pac::Interrupt::AUDIO_CODEC => {
            let audio_codec = unsafe { &*pac::AUDIO_CODEC::PTR };

            // get pending interrupts
            let pending = audio_codec.ac_dac_fifos.read();

            // clear all pending interrupts
            if pending.txe_int().is_pending() {
                audio_codec.ac_dac_fifos.write(|w| w.txe_int().set_bit());
                while audio_codec.ac_dac_fifos.read().txe_int().bit_is_set() {}
            }
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

            // dump some debug info
            let counter = unsafe { COUNTER };
            if counter % 30000 == 0 {
                let count = audio_codec.ac_dac_cnt.read().bits();
                println!("ac_dac_cnt: {} / {}", count, count / 48_000);
            }
        }
        x => {
            println!("Unexpected claim: {:?}", x);
        }
    }

    unsafe { COUNTER += 1 };

    // Release claim
    plic.complete(claim);
}
