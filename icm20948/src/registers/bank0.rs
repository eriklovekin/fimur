use core::marker::PhantomData;
use super::{Register, Bank0};

pub const WHO_AM_I: Register<Bank0> = Register {
    name: "WHO_AM_I",
    addr: 0x00,
    _bank: PhantomData,
};

pub const USER_CTRL: Register<Bank0> = Register {
    name: "USER_CTRL",
    addr: 0x03,
    _bank: PhantomData,
};

pub const LP_CONFIG: Register<Bank0> = Register {
    name: "LP_CONFIG",
    addr: 0x05,
    _bank: PhantomData,
};

pub const PWR_MGMT_1: Register<Bank0> = Register {
    name: "PWR_MGMT_1",
    addr: 0x06,
    _bank: PhantomData,
};

pub const PWR_MGMT_2: Register<Bank0> = Register {
    name: "PWR_MGMT_2",
    addr: 0x07,
    _bank: PhantomData,
};

pub const INT_PIN_CFG: Register<Bank0> = Register {
    name: "INT_PIN_CFG",
    addr: 0x0F,
    _bank: PhantomData,
};

pub const INT_ENABLE: Register<Bank0> = Register {
    name: "INT_ENABLE",
    addr: 0x10,
    _bank: PhantomData,
};

pub const INT_ENABLE_1: Register<Bank0> = Register {
    name: "INT_ENABLE_1",
    addr: 0x11,
    _bank: PhantomData,
};

pub const INT_ENABLE_2: Register<Bank0> = Register {
    name: "INT_ENABLE_2",
    addr: 0x12,
    _bank: PhantomData,
};

pub const INT_ENABLE_3: Register<Bank0> = Register {
    name: "INT_ENABLE_3",
    addr: 0x13,
    _bank: PhantomData,
};

pub const I2C_MST_STATUS: Register<Bank0> = Register {
    name: "I2C_MST_STATUS",
    addr: 0x17,
    _bank: PhantomData,
};

pub const INT_STATUS: Register<Bank0> = Register {
    name: "INT_STATUS",
    addr: 0x19,
    _bank: PhantomData,
};

pub const INT_STATUS_1: Register<Bank0> = Register {
    name: "INT_STATUS_1",
    addr: 0x1A,
    _bank: PhantomData,
};

pub const INT_STATUS_2: Register<Bank0> = Register {
    name: "INT_STATUS_2",
    addr: 0x1B,
    _bank: PhantomData,
};

pub const INT_STATUS_3: Register<Bank0> = Register {
    name: "INT_STATUS_3",
    addr: 0x1C,
    _bank: PhantomData,
};

pub const DELAY_TIMEH: Register<Bank0> = Register {
    name: "DELAY_TIMEH",
    addr: 0x28,
    _bank: PhantomData,
};

pub const DELAY_TIMEL: Register<Bank0> = Register {
    name: "DELAY_TIMEL",
    addr: 0x29,
    _bank: PhantomData,
};

pub const ACCEL_XOUT_H: Register<Bank0> = Register {
    name: "ACCEL_XOUT_H",
    addr: 0x2D,
    _bank: PhantomData,
};

pub const ACCEL_XOUT_L: Register<Bank0> = Register {
    name: "ACCEL_XOUT_L",
    addr: 0x2E,
    _bank: PhantomData,
};

pub const ACCEL_YOUT_H: Register<Bank0> = Register {
    name: "ACCEL_YOUT_H",
    addr: 0x2F,
    _bank: PhantomData,
};

pub const ACCEL_YOUT_L: Register<Bank0> = Register {
    name: "ACCEL_YOUT_L",
    addr: 0x30,
    _bank: PhantomData,
};

pub const ACCEL_ZOUT_H: Register<Bank0> = Register {
    name: "ACCEL_ZOUT_H",
    addr: 0x31,
    _bank: PhantomData,
};

pub const ACCEL_ZOUT_L: Register<Bank0> = Register {
    name: "ACCEL_ZOUT_L",
    addr: 0x32,
    _bank: PhantomData,
};

pub const GYRO_XOUT_H: Register<Bank0> = Register {
    name: "GYRO_XOUT_H",
    addr: 0x33,
    _bank: PhantomData,
};

pub const GYRO_XOUT_L: Register<Bank0> = Register {
    name: "GYRO_XOUT_L",
    addr: 0x34,
    _bank: PhantomData,
};

pub const GYRO_YOUT_H: Register<Bank0> = Register {
    name: "GYRO_YOUT_H",
    addr: 0x35,
    _bank: PhantomData,
};

pub const GYRO_YOUT_L: Register<Bank0> = Register {
    name: "GYRO_YOUT_L",
    addr: 0x36,
    _bank: PhantomData,
};

pub const GYRO_ZOUT_H: Register<Bank0> = Register {
    name: "GYRO_ZOUT_H",
    addr: 0x37,
    _bank: PhantomData,
};

pub const GYRO_ZOUT_L: Register<Bank0> = Register {
    name: "GYRO_ZOUT_L",
    addr: 0x38,
    _bank: PhantomData,
};

pub const TEMP_OUT_H: Register<Bank0> = Register {
    name: "TEMP_OUT_H",
    addr: 0x39,
    _bank: PhantomData,
};

pub const TEMP_OUT_L: Register<Bank0> = Register {
    name: "TEMP_OUT_L",
    addr: 0x3A,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_00: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_00",
    addr: 0x3B,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_01: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_01",
    addr: 0x3C,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_02: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_02",
    addr: 0x3D,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_03: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_03",
    addr: 0x3E,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_04: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_04",
    addr: 0x3F,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_05: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_05",
    addr: 0x40,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_06: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_06",
    addr: 0x41,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_07: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_07",
    addr: 0x42,
    _bank: PhantomData,
};

pub const EXT_SLV_SENS_DATA_08: Register<Bank0> = Register {
    name: "EXT_SLV_SENS_DATA_08",
    addr: 0x43,
    _bank: PhantomData,
};
