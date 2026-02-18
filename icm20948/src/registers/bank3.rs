#![allow(dead_code)]
use super::{Register, Bank3};

pub const I2C_MST_ODR_CONFIG: Register<Bank3> =
    Register::new("I2C_MST_ODR_CONFIG", 0x00);

pub const I2C_MST_CTRL: Register<Bank3> =
    Register::new("I2C_MST_CTRL", 0x01);

pub const I2C_MST_DELAY_CTRL: Register<Bank3> =
    Register::new("I2C_MST_DELAY_CTRL", 0x02);

pub const I2C_SLV0_ADDR: Register<Bank3> =
    Register::new("I2C_SLV0_ADDR", 0x03);

pub const I2C_SLV0_REG: Register<Bank3> =
    Register::new("I2C_SLV0_REG", 0x04);

pub const I2C_SLV0_CTRL: Register<Bank3> =
    Register::new("I2C_SLV0_CTRL", 0x05);

pub const I2C_SLV0_DO: Register<Bank3> =
    Register::new("I2C_SLV0_DO", 0x06);

pub const I2C_SLV1_ADDR: Register<Bank3> =
    Register::new("I2C_SLV1_ADDR", 0x07);

pub const I2C_SLV1_REG: Register<Bank3> =
    Register::new("I2C_SLV1_REG", 0x08);

pub const I2C_SLV1_CTRL: Register<Bank3> =
    Register::new("I2C_SLV1_CTRL", 0x09);

pub const I2C_SLV2_DO: Register<Bank3> =
    Register::new("I2C_SLV2_DO", 0x0A);

pub const I2C_SLV2_ADDR: Register<Bank3> =
    Register::new("I2C_SLV2_ADDR", 0x0B);

pub const I2C_SLV2_REG: Register<Bank3> =
    Register::new("I2C_SLV2_REG", 0x0C);

pub const I2C_SLV2_CTRL: Register<Bank3> =
    Register::new("I2C_SLV2_CTRL", 0x0D);

pub const I2C_SLV2_DO_2: Register<Bank3> =
    Register::new("I2C_SLV2_DO", 0x0E);

pub const I2C_SLV3_ADDR: Register<Bank3> =
    Register::new("I2C_SLV3_ADDR", 0x0F);

pub const I2C_SLV3_REG: Register<Bank3> =
    Register::new("I2C_SLV3_REG", 0x10);

pub const I2C_SLV3_CTRL: Register<Bank3> =
    Register::new("I2C_SLV3_CTRL", 0x11);

pub const I2C_SLV3_DO: Register<Bank3> =
    Register::new("I2C_SLV3_DO", 0x12);

pub const I2C_SLV4_ADDR: Register<Bank3> =
    Register::new("I2C_SLV4_ADDR", 0x13);

pub const I2C_SLV4_REG: Register<Bank3> =
    Register::new("I2C_SLV4_REG", 0x14);

pub const I2C_SLV4_CTRL: Register<Bank3> =
    Register::new("I2C_SLV4_CTRL", 0x15);

pub const I2C_SLV4_DO: Register<Bank3> =
    Register::new("I2C_SLV4_DO", 0x16);

pub const I2C_SLV4_DI: Register<Bank3> =
    Register::new("I2C_SLV4_DI", 0x17);

pub const REG_BANK_SEL: Register<Bank3> =
    Register::new("REG_BANK_SEL", 0x7F);