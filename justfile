
default:
    @just --list

all: build py-build

build:
    cargo build -p fimur -p icm20948 -p imu-traits --target riscv32imac-unknown-none-elf

py-build:
    cd fusion-py && maturin develop

clean:
    cargo clean
    cd fusion-py && cargo clean