#![allow(dead_code)]
use super::{Register, Bank2};

pub const GYRO_SMPLRT_DIV: Register<Bank2> =
    Register::new("GYRO_SMPLRT_DIV", 0x00);

pub const GYRO_CONFIG_1: Register<Bank2> =
    Register::new("GYRO_CONFIG_1", 0x01);

pub const GYRO_CONFIG_2: Register<Bank2> =
    Register::new("GYRO_CONFIG_2", 0x02);

pub const XG_OFFS_USRH: Register<Bank2> =
    Register::new("XG_OFFS_USRH", 0x03);

pub const XG_OFFS_USRL: Register<Bank2> =
    Register::new("XG_OFFS_USRL", 0x04);

pub const YG_OFFS_USRH: Register<Bank2> =
    Register::new("YG_OFFS_USRH", 0x05);

pub const YG_OFFS_USRL: Register<Bank2> =
    Register::new("YG_OFFS_USRL", 0x06);

pub const ZG_OFFS_USRH: Register<Bank2> =
    Register::new("ZG_OFFS_USRH", 0x07);

pub const ZG_OFFS_USRL: Register<Bank2> =
    Register::new("ZG_OFFS_USRL", 0x08);

pub const ODR_ALIGN_EN: Register<Bank2> =
    Register::new("ODR_ALIGN_EN", 0x09);

pub const ACCEL_SMPLRT_DIV_1: Register<Bank2> =
    Register::new("ACCEL_SMPLRT_DIV_1", 0x10);

pub const ACCEL_SMPLRT_DIV_2: Register<Bank2> =
    Register::new("ACCEL_SMPLRT_DIV_2", 0x11);

pub const ACCEL_INTEL_CTRL: Register<Bank2> =
    Register::new("ACCEL_INTEL_CTRL", 0x12);

pub const ACCEL_WOM_THR: Register<Bank2> =
    Register::new("ACCEL_WOM_THR", 0x13);

pub const ACCEL_CONFIG: Register<Bank2> =
    Register::new("ACCEL_CONFIG", 0x14);

pub const ACCEL_CONFIG_2: Register<Bank2> =
    Register::new("ACCEL_CONFIG_2", 0x15);

pub const FSYNC_CONFIG: Register<Bank2> =
    Register::new("FSYNC_CONFIG", 0x52);

pub const TEMP_CONFIG: Register<Bank2> =
    Register::new("TEMP_CONFIG", 0x53);

pub const MOD_CTRL_USR: Register<Bank2> =
    Register::new("MOD_CTRL_USR", 0x54);

pub const REG_BANK_SEL: Register<Bank2> =
    Register::new("REG_BANK_SEL", 0x7F);
