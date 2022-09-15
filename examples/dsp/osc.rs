#![allow(dead_code)]
#[allow(unused_imports)]
use super::wavetable;

// - wavetable oscillator -----------------------------------------------------

pub struct Wavetable {
    pub dx: f32, // frequency * (1 / fs)
    phasor: f32,
    shape: Shape,
}

pub enum Shape {
    Sin,
    Saw,
}

impl Wavetable {
    pub const fn new(shape: Shape) -> Wavetable {
        Wavetable {
            shape: shape,
            dx: 0.004_988_662_f32, // 220Hz @ fs=44100KHz
            phasor: 0.,
        }
    }

    pub fn step(&mut self) -> f32 {
        let wt = match self.shape {
            Shape::Saw => &wavetable::SAW,
            Shape::Sin => &wavetable::SIN,
        };

        let wt_index = self.phasor * wavetable::LENGTH as f32;
        let signal = wt[wt_index as usize];

        self.phasor += self.dx;
        if self.phasor >= 1.0_f32 {
            self.phasor -= 1.0_f32;
        }

        signal
    }
}
