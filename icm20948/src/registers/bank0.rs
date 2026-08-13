#![allow(dead_code)]
use super::{Register, Bank0};

pub const WHO_AM_I: Register<Bank0> =
    Register::new("WHO_AM_I", 0x00);

pub const USER_CTRL: Register<Bank0> =
    Register::new("USER_CTRL", 0x03);

pub const LP_CONFIG: Register<Bank0> =
    Register::new("LP_CONFIG", 0x05);

pub const PWR_MGMT_1: Register<Bank0> =
    Register::new("PWR_MGMT_1", 0x06);

pub const PWR_MGMT_2: Register<Bank0> =
    Register::new("PWR_MGMT_2", 0x07);

pub const INT_PIN_CFG: Register<Bank0> =
    Register::new("INT_PIN_CFG", 0x0F);

pub const INT_ENABLE: Register<Bank0> =
    Register::new("INT_ENABLE", 0x10);

pub const INT_ENABLE_1: Register<Bank0> =
    Register::new("INT_ENABLE_1", 0x11);

pub const INT_ENABLE_2: Register<Bank0> =
    Register::new("INT_ENABLE_2", 0x12);

pub const INT_ENABLE_3: Register<Bank0> =
    Register::new("INT_ENABLE_3", 0x13);

pub const I2C_MST_STATUS: Register<Bank0> =
    Register::new("I2C_MST_STATUS", 0x17);

pub const INT_STATUS: Register<Bank0> =
    Register::new("INT_STATUS", 0x19);

pub const INT_STATUS_1: Register<Bank0> =
    Register::new("INT_STATUS_1", 0x1A);

pub const INT_STATUS_2: Register<Bank0> =
    Register::new("INT_STATUS_2", 0x1B);

pub const INT_STATUS_3: Register<Bank0> =
    Register::new("INT_STATUS_3", 0x1C);

pub const DELAY_TIMEH: Register<Bank0> =
    Register::new("DELAY_TIMEH", 0x28);

pub const DELAY_TIMEL: Register<Bank0> =
    Register::new("DELAY_TIMEL", 0x29);

pub const ACCEL_XOUT_H: Register<Bank0> =
    Register::new("ACCEL_XOUT_H", 0x2D);

pub const ACCEL_XOUT_L: Register<Bank0> =
    Register::new("ACCEL_XOUT_L", 0x2E);

pub const ACCEL_YOUT_H: Register<Bank0> =
    Register::new("ACCEL_YOUT_H", 0x2F);

pub const ACCEL_YOUT_L: Register<Bank0> =
    Register::new("ACCEL_YOUT_L", 0x30);

pub const ACCEL_ZOUT_H: Register<Bank0> =
    Register::new("ACCEL_ZOUT_H", 0x31);

pub const ACCEL_ZOUT_L: Register<Bank0> =
    Register::new("ACCEL_ZOUT_L", 0x32);

pub const GYRO_XOUT_H: Register<Bank0> =
    Register::new("GYRO_XOUT_H", 0x33);

pub const GYRO_XOUT_L: Register<Bank0> =
    Register::new("GYRO_XOUT_L", 0x34);

pub const GYRO_YOUT_H: Register<Bank0> =
    Register::new("GYRO_YOUT_H", 0x35);

pub const GYRO_YOUT_L: Register<Bank0> =
    Register::new("GYRO_YOUT_L", 0x36);

pub const GYRO_ZOUT_H: Register<Bank0> =
    Register::new("GYRO_ZOUT_H", 0x37);

pub const GYRO_ZOUT_L: Register<Bank0> =
    Register::new("GYRO_ZOUT_L", 0x38);

pub const TEMP_OUT_H: Register<Bank0> =
    Register::new("TEMP_OUT_H", 0x39);

pub const TEMP_OUT_L: Register<Bank0> =
    Register::new("TEMP_OUT_L", 0x3A);

pub const EXT_SLV_SENS_DATA_00: Register<Bank0> =
    Register::new("EXT_SLV_SENS_DATA_00", 0x3B);

pub const EXT_SLV_SENS_DATA_01: Register<Bank0> =
    Register::new("EXT_SLV_SENS_DATA_01", 0x3C);

pub const EXT_SLV_SENS_DATA_02: Register<Bank0> =
    Register::new("EXT_SLV_SENS_DATA_02", 0x3D);

pub const EXT_SLV_SENS_DATA_03: Register<Bank0> =
    Register::new("EXT_SLV_SENS_DATA_03", 0x43);
