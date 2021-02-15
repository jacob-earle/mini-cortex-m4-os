# mini-cortex-m4-os
A mini OS targeting Cortex-m4 platform (STM32F4-Discovery) written in Rust.

## Dependencies
- xPack QEMU fork: https://xpack.github.io/qemu-arm/
- Add the directory containing the patched QEMU to PATH.
- Nightly rust (cargo) tool chain.
- `rustup target add thumbv7em-none-eabi`

## Run
- `cargo build` to compile the binary image.
- `cargo run` to run the image in QEMU.
