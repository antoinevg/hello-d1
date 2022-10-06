#![cfg_attr(rustfmt, rustfmt_skip)]

use core::ptr::NonNull;

use d1_pac as pac;
use pac::{CCU, GPIO, TWI0};
use pac::twi::twi_stat::STA_A as Status;

use crate::println;

const I2C_CONTROLLER_WRITE: u8 = 0;
const I2C_CONTROLLER_READ: u8 = 1;

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

    /// Write `bytes` to TWI Peripheral at the optional `address`.
    ///
    /// TODO support > 8 bit data address
    pub fn write(&self, device_id: u8, address: Option<u8>, bytes: &[u8]) -> Result<(), I2cError> {
        write_twi0(&self.twi0, device_id, address, bytes)
    }

    /// Read bytes from TWI Peripheral into `buffer`
    ///
    /// TODO this hasn't been tested yet, use at your own peril!
    /// TODO support > 8 bit data address
    pub fn read(&self, device_id: u8, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
        read_twi0(&self.twi0, device_id, address, buffer)
    }
}

/// I2cError
#[derive(Debug)]
pub enum I2cError {
    Timeout,
    Status(Status),
}

// ----------------------------------------------------------------------------

/// pg. 868 9.1.4.1 Initialization for TWI Engine
fn init_twi0(twi0: &TWI0, gpio: &GPIO, ccu: &CCU) {
    // configure gpio multiplex function as TWI mode (Datasheet pg. 35)
    gpio.pb_cfg0.modify(|_, w| {
        w.pb2_select().twi0_sda()  // PB2  twi0-sda
    });
    gpio.pb_cfg1.modify(|_, w| {
        w.pb10_select().twi0_sck() // PB10 twi0-sck
    });

    // open twi0 bus clock gating and de-assert bus reset
    ccu.twi_bgr.modify(|_, w| unsafe {
        w.twi0_gating().pass()
         .twi0_rst().deassert()
    });

    // configure clock rate for 100 kHz standard speed
    //
    // Fin  = APB Clock = 24 MHz
    // F0   = Fin / 2^CLK_N
    // F1   = F0 / 10 * (CLK_M + 1)
    // Fscl = F1 / 10 = Fin / (2^CLK_N * (CLK_M + 1) * 10)
    //
    // 400kHz full speed 2-wire, CLK_N = 1, CLK_M = 2
    //   F0 = 24MHz / 2^1 = 12 MHz
    //   F1 = F0 / 10 * (2 + 1)) = 12 MHz / 30 = 0.4 MHz
    //
    // 100kHz full speed 2-wire, CLK_N = 1, CLK_M = 11
    //   F0 = 24MHz / 2^1 = 12 MHz
    //   F1 = F0 / 10 * (11 + 1)) = 12 MHz / 120 = 0.1 MHz
    twi0.twi_ccr.write(|w| unsafe {
        w.clk_duty().p50() // duty cycle: 50%
         .clk_m().bits(11) // clk_m: 11
         .clk_n().bits(1)  // clk_n: 1
    });

    // 6. configure TWI control register
    twi0.twi_cntr.write(|w| {
        w.bus_en().respond()  // bus enable: set to '1' for operation as controller
         .int_en().low()      // interrupt: disabled
    });
}

#[inline(always)]
fn clear_interrupt_twi0(twi0: &TWI0) {
    twi0.twi_cntr.modify(|_, w| w.int_flag().set_bit()); // clear int flag
}

#[inline(always)]
fn wait_for_interrupt_twi0(twi0: &TWI0) -> Result<Status, I2cError> {
    let mut count = 0xffff;
    while twi0.twi_cntr.read().int_flag().bit_is_clear() {
        if count == 0 {
            println!("wfi_twi0 timeout");
            return Err(I2cError::Timeout);
        } else {
            count -= 1;
        }
    }
    status_twi0(twi0)
}

#[inline(always)]
fn status_twi0(twi0: &TWI0) -> Result<Status, I2cError> {
    let status = twi0.twi_stat.read().sta().variant();
    match status {
        Some(status) => Ok(status),
        None => {
            println!("invalid status");
            Err(I2cError::Status(Status::NRSI))
        }
    }
}


/// pg. 869 9.1.4.2 Writing Data Operation for TWI Engine
fn write_twi0(twi0: &TWI0, device_id: u8, address: Option<u8>, buffer: &[u8]) -> Result<(), I2cError> {
    // 1. clear TWI_EFR then transmit START
    twi0.twi_efr.reset();
    twi0.twi_srst.write(|w| w.soft_rst().set_bit());
    twi0.twi_cntr.modify(|_, w| w.m_sta().set_bit());

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::SCT { // I2C_START_TRANSMIT
        println!("failed to start transmit: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 2. write device id
    let data = ((device_id & 0xff) << 1) | (I2C_CONTROLLER_WRITE & 1);
    twi0.twi_data.write(|w| unsafe { w.bits(data as u32) });
    clear_interrupt_twi0(twi0);

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::AWBT_AR { // I2C_ADDRWRITE_ACK
        println!("failed to send device id: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 3. write data address
    if let Some(address) = address {
        twi0.twi_data.write(|w| unsafe { w.bits(address as u32) });
        clear_interrupt_twi0(twi0);

        // wait for interrupt
        let status = wait_for_interrupt_twi0(twi0)?;
        if status != Status::DBTM_AR { // I2C_DATAWRITE_ACK
            println!("failed to send data address: {:?}", status);
            return Err(I2cError::Status(status));
        }
    }

    // 4. write data
    for &byte in buffer {
        twi0.twi_data.write(|w| unsafe { w.bits(byte as u32) });
        clear_interrupt_twi0(twi0);

        // wait for interrupt
        let status = wait_for_interrupt_twi0(twi0)?;
        if status != Status::DBTM_AR { // I2C_DATAWRITE_ACK
            println!("failed to send data byte '{:0x}': {:?}", byte, status);
            return Err(I2cError::Status(status));
        }
    }

    // 5.  transmit STOP to end write operation
    twi0.twi_cntr.modify(|_, w| w.m_stp().set_bit());
    clear_interrupt_twi0(twi0);

    // wait for STOP operation to complete
    while twi0.twi_cntr.read().m_stp().bit_is_set() {}
    let mut count = 0xffff;
    while status_twi0(twi0)? != Status::NRSI {
        if count == 0 {
            println!("timed out waiting for STOP operation to complete");
            return Err(I2cError::Timeout);
        } else {
            count -= 1;
        }
    }

    Ok(())
}

/// pg. 869 9.1.4.3 Reading Data Operation for TWI Engine
fn read_twi0(twi0: &TWI0, device_id: u8, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
    // 1. clear TWI_EFR, set ACK to 1 and transmit START
    twi0.twi_efr.reset();
    twi0.twi_cntr.modify(|_, w| {
        w.a_ack().set_bit()
         .m_sta().set_bit()
    });

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::SCT { // I2C_START_TRANSMIT
        println!("failed to start transmit: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 2. write device id
    let data = ((device_id & 0xff) << 1) | (I2C_CONTROLLER_WRITE & 1);
    twi0.twi_data.write(|w| unsafe { w.bits(data as u32) });
    clear_interrupt_twi0(twi0);

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::AWBT_AR { // I2C_ADDRWRITE_ACK
        println!("failed to send device id: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 3. write data address
    twi0.twi_data.write(|w| unsafe { w.bits(address as u32) });
    clear_interrupt_twi0(twi0);

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::DBTM_AR { // I2C_DATAWRITE_ACK
        println!("failed to send data address: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 4. transmit RESTART
    twi0.twi_cntr.modify(|_, w| {
        w.m_sta().set_bit()
         .int_flag().set_bit() // clear interrupt
    });
    // clear_interrupt_twi0(twi0);

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::RSCT { // I2C_RESTART_TRANSMIT
        println!("failed to start transmit: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 4. write device id to start read operation
    let data = ((device_id & 0xff) << 1) | (I2C_CONTROLLER_READ & 1);
    twi0.twi_data.write(|w| unsafe { w.bits(data as u32) });
    clear_interrupt_twi0(twi0);

    // wait for interrupt
    let status = wait_for_interrupt_twi0(twi0)?;
    if status != Status::ARBT_AR { // I2C_ADDRREAD_ACK
        println!("failed to start read operation: {:?}", status);
        return Err(I2cError::Status(status));
    }

    // 5. receive data
    let last_byte = buffer.len() - 1;
    for (index, byte) in buffer.iter_mut().enumerate() {
        if index == last_byte {
            twi0.twi_cntr.modify(|_, w| {
                w.a_ack().clear_bit()    // don't send ack for last byte
                 .int_flag().set_bit()   // clear interrupt
            });
        } else {
            twi0.twi_cntr.modify(|_, w| {
                w.a_ack().set_bit()      // send ack
                 .int_flag().set_bit()   // clear interrupt
            });
        }
        //clear_interrupt_twi0(twi0);

        // wait for interrupt to signal next byte
        wait_for_interrupt_twi0(twi0);

        // read byte
        *byte = twi0.twi_data.read().bits() as u8;

        // check status
        let mut count = 0xffff;
        while status_twi0(twi0)? != Status::DBRM_AT { // I2C_DATAREAD_ACK
            if count == 0 {
                println!("timed out waiting for STOP operation to complete");
                return Err(I2cError::Timeout);
            } else {
                count -= 1;
            }
        }
    }

    // 6. transmit STOP to end read operation
    twi0.twi_cntr.modify(|_, w| w.m_stp().set_bit());
    clear_interrupt_twi0(twi0);

    // wait for STOP operation to complete
    while twi0.twi_cntr.read().m_stp().bit_is_set() {}
    let mut count = 0xffff;
    while status_twi0(twi0)? != Status::NRSI {
        if count == 0 {
            println!("timed out waiting for STOP operation to complete");
            return Err(I2cError::Timeout);
        } else {
            count -= 1;
        }
    }

    Ok(())
}
