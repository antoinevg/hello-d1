pub mod pins {
    pub struct PE8;

    impl PE8 {
        pub fn is_low(&self) -> bool {
            true
        }
    }
}

pub trait Button {
    fn is_pressed(&self) -> bool;
}

impl Button for pins::PE8 {
    fn is_pressed(&self) -> bool {
        self.is_low()
    }
}

pub fn new(pin: pins::PE8) -> pins::PE8 {
    pins::PE8
}
