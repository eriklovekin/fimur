
default:
    @just --list

all: build pybuild

clean:
    cargo clean
    cd fusion-py && cargo clean

build:
    cargo build -p fimur -p icm20948 -p imu-traits --target riscv32imac-unknown-none-elf

pybuild:
    cd fusion-py && maturin develop

run:
    cargo run -p fimur --target riscv32imac-unknown-none-elf