#![allow(dead_code)]
use super::{Register, Bank1};

pub const SELF_TEST_X_GYRO: Register<Bank1> =
    Register::new("SELF_TEST_X_GYRO", 0x02);

pub const SELF_TEST_Y_GYRO: Register<Bank1> =
    Register::new("SELF_TEST_Y_GYRO", 0x03);

pub const SELF_TEST_Z_GYRO: Register<Bank1> =
    Register::new("SELF_TEST_Z_GYRO", 0x04);

pub const SELF_TEST_X_ACCEL: Register<Bank1> =
    Register::new("SELF_TEST_X_ACCEL", 0x0E);

pub const SELF_TEST_Y_ACCEL: Register<Bank1> =
    Register::new("SELF_TEST_Y_ACCEL", 0x0F);

pub const SELF_TEST_Z_ACCEL: Register<Bank1> =
    Register::new("SELF_TEST_Z_ACCEL", 0x10);

pub const XA_OFFS_H: Register<Bank1> =
    Register::new("XA_OFFS_H", 0x14);

pub const XA_OFFS_L: Register<Bank1> =
    Register::new("XA_OFFS_L", 0x15);

pub const YA_OFFS_H: Register<Bank1> =
    Register::new("YA_OFFS_H", 0x17);

pub const YA_OFFS_L: Register<Bank1> =
    Register::new("YA_OFFS_L", 0x18);

pub const ZA_OFFS_H: Register<Bank1> =
    Register::new("ZA_OFFS_H", 0x1A);

pub const ZA_OFFS_L: Register<Bank1> =
    Register::new("ZA_OFFS_L", 0x1B);

pub const TIMEBASE_CORRECTION_PLL: Register<Bank1> =
    Register::new("TIMEBASE_CORRECTION_PLL", 0x28);

pub const REG_BANK_SEL: Register<Bank1> =
    Register::new("REG_BANK_SEL", 0x7F);
