#![feature(naked_functions, asm_sym, asm_const)]

#![no_std]
#![no_main]

use core::arch::asm;
use core::panic::PanicInfo;


// - .text - flash header -----------------------------------------------------

#[naked]
#[link_section = ".head.text"]
#[export_name = "head_jump"]
pub unsafe extern "C" fn head_jump() {
    asm!(
        ".option push",
        ".option rvc",
        "c.j    0x60",
        ".option pop",
        // sym start,
        options(noreturn)
    )
}


// - .text - entry point ------------------------------------------------------

#[naked]
#[link_section = ".text.entry"]
#[export_name = "start"]
pub unsafe extern "C" fn start() -> ! {
    asm!(
        "la     sp, {stack}",
        "li     t0, {per_hart_stack_size}",
        "add    sp, sp, t0",
        "j      {main}",
        per_hart_stack_size = const PER_HART_STACK_SIZE,
        stack = sym SBI_STACK,
        main = sym main,
        options(noreturn)
    )
}


// - .text - constants --------------------------------------------------------

const PER_HART_STACK_SIZE: usize = 4 * 4096; // 16KiB
const SBI_STACK_SIZE: usize = 1 * PER_HART_STACK_SIZE;


// - .data --------------------------------------------------------------------

#[repr(C)]
pub struct HeadData {
    magic: [u8; 8],
    checksum: u32, // filled by blob generator
    length: u32,   // filled by blob generator
    pub_head_size: u32,
    fel_script_address: u32,
    fel_uenv_length: u32,
    dt_name_offset: u32,
    dram_size: u32,
    boot_media: u32,
    string_pool: [u32; 13],
}

#[link_section = ".head.data"]
pub static HEAD_DATA: HeadData = HeadData {
    magic: *b"eGON.BT0",
    checksum: 0,
    length: 0,
    pub_head_size: 0,
    fel_script_address: 0,
    fel_uenv_length: 0,
    dt_name_offset: 0,
    dram_size: 0,
    boot_media: 0,
    string_pool: [0; 13],
};


// - .bss - static global state -----------------------------------------------

#[link_section = ".bss.uninit"]
static mut SBI_STACK: [u8; SBI_STACK_SIZE] = [0; SBI_STACK_SIZE];


// - .bss - main --------------------------------------------------------------

extern "C" fn main() {
    unsafe { asm!("la a0, {}", sym HEAD_DATA) };

    init_bss();

    let p = d1_pac::Peripherals::take().unwrap();
    let uart = p.UART0;

    let hello = "🦀 Hello Lichee RV D1!\n";
    for b in hello.bytes() {
        uart.thr().write(|w| unsafe { w.thr().bits(b) });
    }

    loop {
    }
}

fn init_bss() {
    extern "C" {
        static mut ebss: u32;
        static mut sbss: u32;
        static mut edata: u32;
        static mut sdata: u32;
        static sidata: u32;
    }
    unsafe {
        r0::zero_bss(&mut sbss, &mut ebss);
        r0::init_data(&mut sdata, &mut edata, &sidata);
    }
}

#[cfg_attr(not(test), panic_handler)]
#[allow(unused)]
fn panic(info: &PanicInfo) -> ! {
    loop {}
}
