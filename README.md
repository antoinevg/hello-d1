# hello-d1

A starter project for the Lichee RV D1. There are many like it, but this one is mine.

## Dependencies

### Rust

    rustup target add --toolchain nightly riscv64imac-unknown-none-elf
    rustup component add --toolchain nightly llvm-tools-preview
    cargo install cargo-binutils

### xfel

Build:

    git clone https://github.com/xboot/xfel.git xfel.git
    cd xfel.git
    make

Put device into xfel mode:

* Power off
* Hold xfel button
* Power on
* Release xfel button

Test:

    ./xfel version



## Debugging

### Dock UART

    picocom --imap lfcrlf -b 115200 /dev/cu.usbmodem224302
    # ctrl-a + ctrl-x to exit

### D1 UART

    picocom --imap lfcrlf -b 115200 /dev/cu.usbmodem22441202

### TODO gdb

Install via homebrew:

    brew tap "riscv-software-src/riscv"
    brew install riscv-tools

Build from source:

    https://github.com/T-head-Semi/xuantie-gnu-toolchain



## Build, flash & run

    cargo build --release --example audio_testsignal

    rust-objcopy target/riscv64imac-unknown-none-elf/debug/test-d1-flash-bare \
        --binary-architecture=riscv64 \
        --strip-all -O binary \
        target/flash.bin

    # put device into xfel mode

    xfel ddr d1
    xfel write 0x40000000 target/flash.bin
    xfel exec 0x40000000


## Noisy Audio Output

Add a [RC Low Pass Filter](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-low-pass-and-high-pass-filter) with -3dB Cutoff @ ~33.8kHz:

    C = 100nF
    R = 47R


## Greets fly to

* [riscv-rust / riscv-rust-quickstart](https://github.com/riscv-rust/riscv-rust-quickstart)
* [luojia65 / test-d1-flash-bare](https://github.com/luojia65/test-d1-flash-bare)
* [orangecms / test-d1-flash-bare](https://github.com/orangecms/test-d1-flash-bare/tree/dram-rerere)
* [adamgreig / d1rgb](https://github.com/adamgreig/d1rgb)
* [tosc-rs / d1-playground](https://github.com/tosc-rs/d1-playground)
