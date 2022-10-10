/// I2C Device address when WM8731 CSB pin is high
pub const DEVICE_ID_A: u8 = 0x1a;
/// I2C Device address when WM8731 CSB pin is low
pub const DEVICE_ID_B: u8 = 0x1b;

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum Register {
    LINVOL = 0x00,
    RINVOL = 0x01,
    LHPOUT = 0x02,
    RHPOUT = 0x03,
    APANA  = 0x04,
    APDIGI = 0x05,
    PWR    = 0x06,
    IFACE  = 0x07,
    SRATE  = 0x08,
    ACTIVE = 0x09,
    RESET  = 0x0F,
}

pub const DEFAULT_CONFIG: &[(Register, u16)] = &[
    // power-on codec, reset and switch to inactive
    (Register::PWR,    0b1000_0000),
    (Register::RESET,  0x0),
    (Register::ACTIVE, 0b0),

    // configure power
    (Register::PWR,    0b0001_0000),   // power-down outputs

    // select mic for adc input, dac select
    //(Register::APANA,  0b0_0001_0100),

    // select line for adc input, dac select
    (Register::APANA,  0b0_0001_0000),

    // configure digital routing
    (Register::APDIGI, 0b1_0110),      // store dc-offset, de-emphasis for fs=48kHz, enable hpf

    // configure interface format
    (Register::IFACE,  0b0100_1110),   // i2s_clock_internal, 32bit, i2s

    // configure mode and sample-rate
    (Register::SRATE,  0b0000_0000),   // mode=0, fs=48kHz

    // set input level
    (Register::LINVOL, 0b0_0001_0111), // 0dB
    (Register::RINVOL, 0b0_0001_0111), // 0dB

    // set output level
    (Register::LHPOUT, 0b0_1111_1001), // zero cross detect enable, 0dB
    (Register::RHPOUT, 0b0_1111_1001), // zero cross detect enable, 0dB

    // switch codec to active
    (Register::ACTIVE, 0b1),

    // configure power
    (Register::PWR,    0b0000_0000),   // power-up outputs
];
