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
    LOUT1V = 0x02,
    ROUT1V = 0x03,
    APANA  = 0x04,
    APDIGI = 0x05, // 0000_0101
    PWR    = 0x06,
    IFACE  = 0x07, // 0000_0111
    SRATE  = 0x08, // 0000_1000
    ACTIVE = 0x09, // 0000_1001
    RESET  = 0x0F,
}

pub const DEFAULT_CONFIG: &[(Register, u8)] = &[
    (Register::PWR,    0x80),
    (Register::RESET,  0x00),
    (Register::ACTIVE, 0x00),

    (Register::APANA,  0x12),
    //(Register::APANA,  0b0001_0010), // MICBOOST=0 MUTEMIC=1 INSEL=0 BYPASS=0 DACSEL=1 SIDETONE=0

    (Register::APDIGI, 0x00),
    (Register::PWR,    0x00),

    (Register::IFACE,  0x02),
    //(Register::IFACE,  0b0000_0010), // 0x02 FORMAT=b10 IRL=b00 LRP=0 LRSWAP=0 MS=0 BCKLINV=0
    //(Register::IFACE,  0b0100_0010), // 0x42 FORMAT=b10 IRL=b00 LRP=0 LRSWAP=0 MS=1 BCKLINV=0

    (Register::SRATE,  0b0000_0000), // MODE=0 BOSR=0 FS=48Khz CLKIDIV2=0 CLKODIV2=0
    //(Register::SRATE,  0b0000_0001), // MODE=1 BOSR=0 FS=48Khz CLKIDIV2=0 CLKODIV2=0

    (Register::LINVOL, 0x17),
    (Register::RINVOL, 0x17),
    (Register::LOUT1V, 0x79),  // 0dB
    (Register::ROUT1V, 0x79),  // 0dB
    (Register::ACTIVE, 0x01),
];
