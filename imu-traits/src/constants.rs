//! Physical and sensor-specific constants

use core::f32::consts::PI as importPI;

pub const PI: f32 = importPI;
pub const EARTH_GRAVITY: f32 = 9.80665; // m/s^2

// conversions
pub const DEG_TO_RAD: f32 = PI/180.;
pub const RAD_TO_DEG: f32 = 1./DEG_TO_RAD;
pub const G_TO_MPS2: f32 = EARTH_GRAVITY;

// sensor-specific data
// scale factors called out as LSB/G
pub const ACCEL_FS0_SCALE: f32 = 1./16384.;
pub const ACCEL_FS1_SCALE: f32 = 1./8192.;
pub const ACCEL_FS2_SCALE: f32 = 1./4096.;
pub const ACCEL_FS3_SCALE: f32 = 1./2048.;

// scale factors called out as LSB/dps
pub const GYRO_FS0_SCALE: f32 = 1./131.;
pub const GYRO_FS1_SCALE: f32 = 1./65.5;
pub const GYRO_FS2_SCALE: f32 = 1./32.8;
pub const GYRO_FS3_SCALE: f32 = 1./16.4;