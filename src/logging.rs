use d1_pac as pac;

// TODO https://github.com/riscv-rust/hifive1/blob/master/src/stdout.rs

/// init
pub fn init(p: &pac::Peripherals) {
    // Enable UART0 clock.
    let ccu = &p.CCU;
    ccu.uart_bgr
        .write(|w| w.uart0_gating().pass().uart0_rst().deassert());

    // Set PB8 and PB9 to function 6, UART0, internal pullup.
    let gpio = &p.GPIO;
    gpio.pb_cfg1
        .write(|w| w.pb8_select().uart0_tx().pb9_select().uart0_rx());
    gpio.pb_pull0
        .write(|w| w.pc8_pull().pull_up().pc9_pull().pull_up());

    // Configure UART0 for 115200 8n1.
    // By default APB1 is 24MHz, use divisor 13 for 115200.
    let uart0 = &p.UART0;
    uart0.mcr.write(|w| unsafe { w.bits(0) });
    uart0.fcr().write(|w| w.fifoe().set_bit());
    uart0.halt.write(|w| w.halt_tx().enabled());
    uart0.lcr.write(|w| w.dlab().divisor_latch());
    uart0.dll().write(|w| unsafe { w.dll().bits(13) });
    uart0.dlh().write(|w| unsafe { w.dlh().bits(0) });
    uart0.lcr.write(|w| w.dlab().rx_buffer().dls().eight());
    uart0.halt.write(|w| w.halt_tx().disabled());
}

/// Uart
pub(crate) struct Uart(pub d1_pac::UART0);

impl core::fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.as_bytes() {
            self.0.thr().write(|w| unsafe { w.thr().bits(*byte) });
            while self.0.usr.read().tfnf().bit_is_clear() {}
        }
        Ok(())
    }
}

/// print!
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::logging::_print(core::format_args!($($arg)*));
    }
}

/// println!
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        $crate::logging::_print(core::format_args!($($arg)*));
        $crate::print!("\r\n");
    }
}

/// _print
pub fn _print(args: core::fmt::Arguments) {
    use core::fmt::Write;

    unsafe {
        let uart0 = pac::Peripherals::steal().UART0;
        Uart(uart0).write_fmt(args).ok();
    }
}
