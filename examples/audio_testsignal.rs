#![allow(dead_code, non_snake_case, unused_variables)]
#![no_std]
#![no_main]

mod dsp;

use d1_pac as pac;
use panic_halt as _;
use twiddle;

use hello_d1::audio_codec::{self, BLOCK_LENGTH};
use hello_d1::dmac;
use hello_d1::plic;
use hello_d1::logging;
use hello_d1::println;

#[riscv_rt::entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    // logging
    logging::init(&p);
    println!("🦀 Hello Lichee RV D1!");

    // trim bandgap reference voltage ???
    //audio_codec::trim_bandgap_ref_voltage();

    // gpio - led
    let gpio = &p.GPIO;
    const PC1: u32 = 1;
    gpio.pc_cfg0.write(|w| w.pc1_select().output());

    // ccu
    let mut ccu = p.CCU;

    // audio_codec
    let audio_codec = audio_codec::AudioCodec::new(p.AUDIOCODEC, &mut ccu);

    // dmac
    let mut dmac = dmac::Dmac::new(p.DMAC, &mut ccu);

    // plic
    let plic = plic::Plic::new(p.PLIC);
    unsafe {
        plic.set_priority(pac::Interrupt::DMAC_NS, plic::Priority::P10);
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

static mut COUNTER: usize = 0;

#[export_name = "MachineExternal"]
extern "C" fn MachineExternal() {
    let plic = unsafe { plic::Plic::summon() };
    let gpio = unsafe { &*pac::GPIO::PTR };
    let audio_codec = unsafe { audio_codec::AudioCodec::summon() };

    // Claim interrupt
    let claim = plic.claim();
    //println!("claim: {:?}", claim);

    match claim {
        pac::Interrupt::GPIOE_NS => {
            gpio.pe_eint_status
                .modify(|_r, w| w.eint8_status().set_bit());
            // Wait for the interrupt to clear to avoid repeat interrupts
            while gpio.pe_eint_status.read().eint8_status().bit_is_set() {}
        }
        pac::Interrupt::DMAC_NS => {
            let dmac = unsafe { &*pac::DMAC::PTR };

            // get some stats
            let packages = dmac.dmac_pkg_num_reg2.read().bits();
            let left = dmac.dmac_bcnt_left_reg2.read().bits();
            //let bits = dmac.dmac_sta_reg.read().bits();
            //let mbus = twiddle::bit(bits, 31);
            //let chans = twiddle::range(bits, 0..15);

            // get pending bits
            let bits = dmac.dmac_irq_pend_reg0.read().bits();
            let pending = twiddle::range(bits, 8..10);

            // clear pending interrupts
            if twiddle::bit(pending, 0) { // half package
                dmac.dmac_irq_pend_reg0.write(|w| w.dma2_hlaf_irq_pend().set_bit());
                while dmac.dmac_irq_pend_reg0.read().dma2_hlaf_irq_pend().bit_is_set() {}
            }
            if twiddle::bit(pending, 1) { // end of package
                dmac.dmac_irq_pend_reg0.write(|w| w.dma2_pkg_irq_pend().set_bit());
                while dmac.dmac_irq_pend_reg0.read().dma2_pkg_irq_pend().bit_is_set() {}
            }
            if twiddle::bit(pending, 2) { // end of queue
                dmac.dmac_irq_pend_reg0.write(|w| w.dma2_queue_irq_pend().set_bit());
                while dmac.dmac_irq_pend_reg0.read().dma2_queue_irq_pend().bit_is_set() {}
            }

            // dump some debug info
            let counter = unsafe { COUNTER };
            if counter % 1000 == 0 {
                println!("DMAC_NS pend:{:#05b}", pending); // 0: idle, 1: busy
                println!("DMAC_NS pkg:{}", packages);
                println!("DMAC_NS left:{}\n", left);
                //println!("DMAC_NS mbus:{} chans:{:#018b}", mbus, chans); // 0: idle, 1: busy
            }
        }
        pac::Interrupt::AUDIO_CODEC => {
            let audio_codec = unsafe { &*pac::AUDIOCODEC::PTR };

            // clear all pending interrupts
            let bits = audio_codec.ac_dac_fifos.read().bits();
            if twiddle::bit(bits, 3) {
                let bits = twiddle::set(bits, 3, true);
            }
            if twiddle::bit(bits, 2) {
                let bits = twiddle::set(bits, 2, true);
            }
            if twiddle::bit(bits, 1) {
                let bits = twiddle::set(bits, 1, true);
            }
            audio_codec.ac_dac_fifos.write(|w| unsafe { w.bits(bits) });
            loop {
                let bits = audio_codec.ac_dac_fifos.read().bits();
                if twiddle::range(bits, 1..3) == 0 {
                    break;
                } else {
                    println!("FUCK: {:#034b}", bits);
                }
            }

            //let bits = audio_codec.ac_dac_fifos.read().bits();
            //let bits = twiddle::range(bits, 1..3);
            //println!("ac_dac_fifos: {:#05b}", bits);

            // dump some debug info
            let counter = unsafe { COUNTER };
            if counter % 30000 == 0 {
                let count = audio_codec.ac_dac_cnt.read().bits();
                //println!("ac_dac_cnt: {:#034b} -> {}", count, counter);
                println!("ac_dac_cnt: {}", count);
            }


        }
        x => {
            println!("Unexpected claim: {:?}", x);
            //panic!("unexpected claim");
        }
    }

    unsafe { COUNTER += 1 };

    // Release claim
    plic.complete(claim);
}
