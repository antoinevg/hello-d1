#![allow(dead_code, non_snake_case, unused_variables)]
#![no_std]
#![no_main]

use d1_pac as pac;
use pac::{Interrupt, Peripherals};

use panic_halt as _;

use hello_d1::logging;
use hello_d1::plic::{Plic, Priority};
use hello_d1::println;

#[riscv_rt::entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();

    // logging
    logging::init(&p);
    println!("🦀 Hello Lichee RV D1!");

    // gpio - PE8
    let gpio = &p.GPIO;
    const PE8: u32 = 8;
    gpio.pe_cfg1.modify(|_, w| w.pe8_select().pe_eint8());
    gpio.pe_pull0.write(|w| w.pe8_pull().pull_up());
    gpio.pe_eint_cfg1.modify(|_, w| w.eint8_cfg().double_edge());
    gpio.pe_eint_ctl.modify(|_, w| w.eint8_ctl().enable());

    // gpio - led
    const PC1: u32 = 1;
    gpio.pc_cfg0.write(|w| w.pc1_select().output());

    // plic
    let plic = Plic::new(p.PLIC);
    unsafe {
        plic.set_priority(Interrupt::GPIOE_NS, Priority::P1);
        plic.unmask(Interrupt::GPIOE_NS);
    }

    // enable interrupts
    unsafe {
        riscv::interrupt::enable();
        riscv::register::mie::set_mext();
    }

    loop {
        unsafe {
            riscv::asm::wfi();
        }

        // read button
        let pe8 = gpio.pe_dat.read().bits();
        let is_high = pe8 & (1 << PE8) != 0;
        if is_high {
            println!("high");
            gpio.pc_dat
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << PC1)) });
        } else {
            println!("low");
            gpio.pc_dat
                .modify(|r, w| unsafe { w.bits(r.bits() & (0 << PC1)) });
        }
    }
}

#[export_name = "GPIOE_NS"]
extern "C" fn GPIOE_NS() {
    println!("GPIOE_NS");
}

#[export_name = "MachineExternal"]
extern "C" fn MachineExternal() {
    let plic = unsafe { Plic::summon() };
    let gpio = unsafe { &*pac::GPIO::PTR };

    // Claim interrupt
    let claim = plic.claim();
    println!("claim: {:?}", claim);

    match claim {
        pac::Interrupt::GPIOE_NS => {
            gpio.pe_eint_status
                .modify(|_r, w| w.eint8_status().set_bit());
            // Wait for the interrupt to clear to avoid repeat interrupts
            while gpio.pe_eint_status.read().eint8_status().bit_is_set() {}
        }
        x => {
            println!("Unexpected claim: {:?}", x);
            panic!("unexpected claim");
        }
    }

    // Release claim
    plic.complete(claim);
}
