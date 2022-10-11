pub mod osc;
pub mod wavetable;


// - float => integer conversions ---------------------------------------------
use core::num::Wrapping;

#[inline(always)]
pub fn f32_to_u16(x: f32) -> u16 {
    let x = x * 32_767.;
    let x = if x > 32_767. {
        32_767.
    } else if x < -32_768. {
        -32_768.
    } else {
        x
    };
    (x as i16) as u16
}

#[inline(always)]
pub fn f32_to_u20(x: f32) -> u32 {
    let x = x * 524_287.;
    let x = if x > 524_287. {
        524_287.
    } else if x < -524_288. {
        -524_288.
    } else {
        x
    };
    (x as i32) as u32
}

#[inline(always)]
pub fn f32_to_u24(x: f32) -> u32 {
    let x = x * 8_388_607.;
    let x = if x > 8_388_607. {
        8_388_607.
    } else if x < -8_388_608. {
        -8_388_608.
    } else {
        x
    };
    (x as i32) as u32
}

#[inline(always)]
pub fn f32_to_u32(x: f32) -> u32 {
    let x = x * 2_147_483_647.;
    let x = if x > 2_147_483_647. {
        2_147_483_647.
    } else if x < -2_147_483_648. {
        -2_147_483_648.
    } else {
        x
    };
    (x as i32) as u32
}

// - integer => float conversions ---------------------------------------------

#[inline(always)]
pub fn u16_to_f32(x: u16) -> f32 {
    let x = (core::num::Wrapping(x) + core::num::Wrapping(0x8000)).0;
    x as f32 / 32768.0 - 1.0
}

#[inline(always)]
pub fn u24_to_f32(y: u32) -> f32 {
    let y = (Wrapping(y) + Wrapping(0x0080_0000)).0 & 0x00FF_FFFF; // convert to i32
    (y as f32 / 8_388_608.) - 1.  // (2^24) / 2
}

#[inline(always)]
pub fn u32_to_f32(y: u32) -> f32 {
    let y = y as i32;
    (y as f32 / 2_147_483_648.) - 1.  // (2^32) / 2
}
