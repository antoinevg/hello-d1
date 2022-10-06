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
    APDIGI = 0x05, // 0000_0101
    PWR    = 0x06,
    IFACE  = 0x07, // 0000_0111
    SRATE  = 0x08, // 0000_1000
    ACTIVE = 0x09, // 0000_1001
    RESET  = 0x0F,
}
/*
pub const DEFAULT_CONFIG: &[(Register, u16)] = &[
    (Register::PWR,    0x80),
    (Register::RESET,  0x00),
    (Register::ACTIVE, 0x00),

    (Register::APANA,  0x12),
    //(Register::APANA,  0b0001_0010), // MICBOOST=0 MUTEMIC=1 INSEL=0 BYPASS=0 DACSEL=1 SIDETONE=0

    (Register::APDIGI, 0x00),
    (Register::PWR,    0x00),

    (Register::IFACE,  0b01001110), // MS=master, IWL=32bit, FORMAT=i2s
    //(Register::IFACE,  0b00001110),   // MS=slave, IWL=32bit, FORMAT=i2s
    //(Register::IFACE,  0b0000_0010), // 0x02 FORMAT=b10 IRL=b00 LRP=0 LRSWAP=0 MS=0 BCKLINV=0
    //(Register::IFACE,  0b0100_0010), // 0x42 FORMAT=b10 IRL=b00 LRP=0 LRSWAP=0 MS=1 BCKLINV=0

    (Register::SRATE,  0b0000_0000), // MODE=0 BOSR=0 FS=48Khz CLKIDIV2=0 CLKODIV2=0
    //(Register::SRATE,  0b0000_0001), // MODE=1 BOSR=0 FS=48Khz CLKIDIV2=0 CLKODIV2=0

    (Register::LINVOL, 0x17),
    (Register::RINVOL, 0x17),
    (Register::LHPOUT, 0x79),  // 0dB
    (Register::RHPOUT, 0x79),  // 0dB
    (Register::ACTIVE, 0x01),
];
*/

pub const DEFAULT_CONFIG: &[(Register, u16)] = &[
    // reset Codec
    (Register::RESET, 0x00),

    // set line inputs 0dB - TODO dev board only has mic in, these are NC
    (Register::LINVOL, 0x17),
    (Register::RINVOL, 0x17),

    // set headphone out to 0dB
    (Register::LHPOUT, 0b0_0111_1001),
    (Register::RHPOUT, 0b0_0111_1001),

    // set analog and digital routing
    (Register::APANA,  0b0_0001_0100), // select mic for adc input, dac select
    (Register::APDIGI, 0b0_0000_0001), // disable hpf

    // configure power management
    //(Register::PWR, 0b0_0100_0000), // clkout power down
    (Register::PWR, 0b0_0000_0000),   // master, so don't power down clock

    // configure digital format
    (Register::IFACE,  0b0_0100_1110), // MS=master, IWL=32bit, FORMAT=i2s
    //(Register::IFACE,  0b0_0000_1110), // MS=slave, IWL=32bit, FORMAT=i2s

    // set samplerate
    (Register::SRATE, 0x00), // fs=48kHz

    (Register::ACTIVE, 0x00),
    (Register::ACTIVE, 0x01),
];
