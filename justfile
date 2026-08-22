
default:
    @just --list

all: build pybuild

build:
    cargo build -p fimur -p icm20948 -p imu-traits --target riscv32imac-unknown-none-elf

pybuild:
    cd fusion-py && maturin develop

clean:
    cargo clean
    cd fusion-py && cargo clean