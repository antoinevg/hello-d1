pub mod osc;
pub mod wavetable;

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
