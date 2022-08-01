#![no_std]
#![allow(non_camel_case_types, unused)] // TODO

pub use d1_pac as pac;

pub mod audio_codec;
pub mod logging;
pub mod plic;
pub mod timer;

mod twiddle {
    pub use core::ops::Range;

    pub trait BitWidth {
        const WIDTH: u8;
        fn width() -> u8;
    }
    impl BitWidth for u32 {
        const WIDTH: u8 = u32::max_value().leading_ones() as u8;
        fn width() -> u8 {
            Self::WIDTH
        }
    }

    #[inline(always)]
    pub fn bit(input: u32, index: u8) -> bool {
        assert!(index < u32::WIDTH);
        input & (1 << index) != 0
    }

    #[inline(always)]
    pub fn set(input: u32, index: u8, value: bool) -> u32 {
        assert!(index < u32::WIDTH);
        let mask = u32::MIN.wrapping_sub(value as u32) ^ input;
        input ^ mask & (1 << index)
    }

    #[inline(always)]
    pub fn range(input: u32, r: Range<u8>) -> u32 {
        assert!(r.start < r.end);
        assert!(r.end < u32::WIDTH);
        // so 1..7 is a Range and it is not inclusive, but yeah potato / potahto
        // worse: Range and RangeInclusive are different types
        // even worse: RangeBounds does not give you the start nor end value 🤦
        // the worst: to express my sadness I think I will now break all the input semantics
        // kids: just because you can argue for your decision does not automatically make it a good decision
        let shift_left = u32::WIDTH - (r.end - 1);
        let shift_right = r.start + shift_left;
        (input << shift_left) >> shift_right
    }

    #[inline(always)]
    pub fn set_range(input: u32, r: Range<u8>, value: u32) -> u32 {
        assert!(r.start < r.end);
        assert!(r.end < u32::WIDTH);
        //let mask = !(range(u32::MAX, r.start..r.end) << r.start);
        let mask = !(u32::MAX >> (u32::WIDTH - r.end) << 1);
        input & mask | value << r.start
    }

    // - operate directly on register -----------------------------------------

    #[inline(always)]
    pub unsafe fn reg_set(p: *mut u32, index: usize, bit: bool) {
        let mask = 1 << index;
        if bit {
            *p |= mask;
        } else {
            *p &= !mask;
        }
    }

    #[inline(always)]
    pub unsafe fn reg_toggle(p: *mut u32, index: usize) {
        let mask = 1 << index;
        *p ^= mask;
    }

    #[inline(always)]
    pub unsafe fn reg_is_set(r: *const u32, index: usize) -> bool {
        (*r & 1 << index) != 0
    }

    #[inline(always)]
    pub unsafe fn reg_is_clear(r: *const u32, index: usize) -> bool {
        (*r & 1 << index) == 0
    }
}
