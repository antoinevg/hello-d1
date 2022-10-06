#![allow(dead_code, non_snake_case, unused_variables)]
#![no_std]
#![no_main]

mod device;
use device::wm8731;
mod dsp;
use dsp::osc;

use d1_pac as pac;
use panic_halt as _;

use hello_d1::twi;
use hello_d1::i2s;
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
    let gpio = p.GPIO;
    const PC1: u32 = 1;
    gpio.pc_cfg0.write(|w| w.pc1_select().output());

    // ccu
    let mut ccu = p.CCU;

    // configure wm8731 over i2c
    println!("configure wm8731");
    let i2c = twi::Twi::new(p.TWI0, &gpio, &ccu);
    let device_id = wm8731::DEVICE_ID_A;
    for (register, value) in wm8731::DEFAULT_CONFIG {
        let register = *register as u8;

        // 7 bit register address + 9 bits of data
        let frame = [
            ((register << 1) & 0xfe) | ((value >> 8) & 0x01) as u8,
            (value & 0xff) as u8,
        ];

        match i2c.write(device_id, None, &frame) {
            Ok(()) => (),
            Err(e) => {
                println!("error configuring wm8731, aborting: {:?}", e);
                break;
            }
        }
        unsafe {
            riscv::asm::delay(50_000);
        }
    }

    // i2s
    let mut i2s = i2s::I2s::new(p.I2S_PCM2, &gpio, &ccu);

    // dmac
    let mut dmac = dmac::Dmac::new(p.DMAC, &mut ccu);

    // plic
    let plic = plic::Plic::new(p.PLIC);
    unsafe {
        plic.set_priority(pac::Interrupt::DMAC_NS, plic::Priority::P1);
        plic.set_priority(pac::Interrupt::I2S_PCM2, plic::Priority::P1);
        plic.unmask(pac::Interrupt::DMAC_NS);
        plic.unmask(pac::Interrupt::I2S_PCM2);
    }

    // enable interrupts
    unsafe {
        riscv::interrupt::enable();
        riscv::register::mie::set_mext();
    }

    // i2s - start
    i2s.start(&mut dmac, unsafe { &TX_BUFFER });

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

/// DMA half-transfer interrupt handler for i2s data
unsafe fn handle_dma_interrupt(half: bool) {
    // oscillator state
    static mut OSC1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    static mut OSC2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
    static mut F1: f32 = 0.;
    static mut F2: f32 = 0.;

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
        //F1 += 0.01;
        if F1 >= 500. {
            F1 = 0.;
        }
        OSC1.dx = (1. / 48_000.) * (220. + F1);
        OSC2.dx = (1. / 48_000.) * (220. + F2);
        let left  = dsp::f32_to_u32(OSC1.step());
        let right = dsp::f32_to_u32(OSC2.step());

        let x = frame_index + skip;
        TX_BUFFER[x + 0] = left as u32;
        TX_BUFFER[x + 1] = right as u32;

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
        pac::Interrupt::I2S_PCM2 => {
            let i2s = unsafe { &*pac::I2S_PCM2::PTR };

            // get pending interrupts
            let pending = i2s.i2s_pcm_ista.read();

            // clear all pending interrupts
            if pending.txe_int().is_pending() { // TODO don't need this
                println!("empty");
                i2s.i2s_pcm_ista.write(|w| w.txe_int().set_bit());
                while i2s.i2s_pcm_ista.read().txe_int().bit_is_set() {}
            }
            if pending.txo_int().is_pending() {
                println!("overrun");
                i2s.i2s_pcm_ista.write(|w| w.txo_int().set_bit());
                while i2s.i2s_pcm_ista.read().txo_int().bit_is_set() {}
            }
            if pending.txu_int().is_pending() {
                println!("underrun");
                i2s.i2s_pcm_ista.write(|w| w.txu_int().set_bit());
                while i2s.i2s_pcm_ista.read().txu_int().bit_is_set() {}
            }
        }
        x => {
            println!("Unexpected claim: {:?}", x);
        }
    }

    // Release claim
    plic.complete(claim);
}
