#![cfg_attr(rustfmt, rustfmt_skip)]

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{CCU, GPIO, TWI0};

use crate::println;

/// Twi Peripheral
pub struct Twi {
    twi0: TWI0,
}

impl Twi {
    /// Create a new `Twi`
    pub fn new(twi0: TWI0, gpio: &GPIO, ccu: &CCU) -> Twi {
        // twi0 initialization
        init_twi0(&twi0, gpio, ccu);

        Self { twi0 }
    }

    pub fn write(&self, device_id: u8, register: u8, buffer: &[u8]) {
        write_twi0(&self.twi0, device_id, register, buffer);
    }

    pub fn read(&self, device_id: u8, register: u8, buffer: &mut [u8]) {
        read_twi0(&self.twi0, device_id, register, buffer);
    }
}

// ----------------------------------------------------------------------------

/// pg. 868 9.1.4.1 Initialization for TWI Engine
fn init_twi0(twi0: &TWI0, gpio: &GPIO, ccu: &CCU) {
    // 1. configure gpio multiplex function as TWI mode (Datasheet pg. 35)
    gpio.pb_cfg0.modify(|_, w| {
        w.pb2_select().twi0_sda()  // PB2  twi0-sda
    });
    gpio.pb_cfg1.modify(|_, w| {
        w.pb10_select().twi0_sck() // PB10 twi0-sck
    });

    // 2,3,4. open twi0 bus clock gating and de-assert bus reset
    ccu.twi_bgr.modify(|_, w| unsafe {
        w.twi0_gating().pass()
         .twi0_rst().deassert()
    });

    // 5. configure clock rate for 100 kHz standard speed
    //
    // Fin  = APB Clock = 24 MHz
    // F0   = Fin / 2^CLK_N
    // F1   = F0 / 10 * (CLK_M + 1)
    // Fscl = F1 / 10 = Fin / (2^CLK_N * (CLK_M + 1) * 10)
    //
    // 400kHz full speed 2-wire, CLK_N = 1, CLK_M = 2
    //   F0 = 24MHz / 2^1 = 12 MHz
    //   F1 = F0 / 10 * (2 + 1)) = 12 MHz / 30 = 0.4 MHz
    // 100kHz full speed 2-wire, CLK_N = 1, CLK_M = 11
    //   F0 = 24MHz / 2^1 = 12 MHz
    //   F1 = F0 / 10 * (11 + 1)) = 12 MHz / 120 = 0.1 MHz
    twi0.twi_ccr.write(|w| unsafe {
        w.clk_duty().p40() // duty cycle: 50%
         .clk_m().bits(0)  // clk_m: 11
         .clk_n().bits(0)  // clk_n: 1
    });

    // 6. configure TWI control register
    twi0.twi_cntr.write(|w| {
        w.int_en().high()     // interrupt enable: ???
         .bus_en().respond()  // bus enable: set to '1' for master operation
    });
}


/// pg. 869 9.1.4.2 Writing Data Operation for TWI Engine
fn write_twi0(twi0: &TWI0, device_id: u8, register: u8, buffer: &[u8]) {
    // 1. clear TWI_EFR then transmit START
    twi0.twi_efr.reset();
    //twi0.twi_efr.write(|w| unsafe { w.bits(0) });
    twi0.twi_cntr.modify(|_, w| w.m_sta().set_bit());

    // 2. wait for interrupt then write device id
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_data.write(|w| unsafe { w.bits(device_id as u32) });

    // 3. wait for interrupt then write device register address
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_data.write(|w| unsafe { w.bits(register as u32) });

    // 4. wait for interrupt then write data
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    for &byte in buffer {
        twi0.twi_data.write(|w| unsafe { w.bits(register as u32) });
        while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    }

    // 5. after data transmission complete transmit STOP to end write operation
    twi0.twi_cntr.modify(|_, w| w.m_stp().set_bit());
}


/// pg. 869 9.1.4.3 Reading Data Operation for TWI Engine
fn read_twi0(twi0: &TWI0, device_id: u8, register: u8, buffer: &mut [u8]) {
    // 1. clear TWI_EFR then set ACK to 1 and transmit START
    twi0.twi_efr.reset();
    //twi0.twi_efr.write(|w| unsafe { w.bits(0) });
    twi0.twi_cntr.modify(|_, w| {
        w.a_ack().set_bit()
         .m_sta().set_bit()
    });

    // 2. wait for interrupt then write device id
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_data.write(|w| unsafe { w.bits(device_id as u32) });

    // 3. wait for interrupt then write device register address
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_data.write(|w| unsafe { w.bits(register as u32) });

    // 4. wait for interrupt then transmit START, wait for interrupt then write device id to start read operation
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_cntr.modify(|_, w| w.m_sta().set_bit());
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
    twi0.twi_data.write(|w| unsafe { w.bits(device_id as u32) });

    // 5. after address transmission each receive will trigger interrupt, read data and when receiving previous interrupt of the last byte data clear A_ACK to stop acknowledge signal of the last byte.
    let last_byte = buffer.len() - 1;
    for (index, byte) in buffer.iter_mut().enumerate() {
        while twi0.twi_cntr.read().int_flag().bit_is_clear() {}
        if index == last_byte {
            twi0.twi_cntr.modify(|_, w| w.a_ack().clear_bit());
        }
        *byte = twi0.twi_data.read().bits() as u8;
    }

    // 6. transmit STOP to end read operation
    twi0.twi_cntr.modify(|_, w| w.m_stp().set_bit());
}
