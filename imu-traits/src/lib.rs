#![no_std]
use heapless::String;

use crate::constants::{EARTH_GRAVITY, RAD_TO_DEG};

pub mod constants;

pub trait Imu {
    type Error;

    /// Require that each implementation include origin and rotation members
    
    /// Returns origin of sensor in F frame [m]
    /// 
    /// Format [xF, yF, zF]
    /// Units: m
    fn origin_f(&self) -> [f32;3];

    // /// Returns rotation vector from S to F frame
    // /// 
    // /// Format: [w, x, y, z], where q = w + xi + yj + zk
    // fn rotation_s2f(&mut self) -> [f32;4];

    /// Returns rotation DCM from S to F frame
    fn rotation_dcm_s2f(&self) -> [[f32;3];3];

    /// Returns most recently measurents from accelerometer and gyroscope
    /// # TODO
    /// Incorporate magnetometer, temperature (and others?) as optional fields
    fn meas(& self) -> Measurement;

    /// Set origin of S frame in F frame
    /// 
    /// Format [xF, yF, zF]
    /// Units: m
    fn set_origin_f(&mut self, origin: [f32;3]);

    /// Set rotation vector from S frame to F frame
    /// 
    /// Format: [w, x, y, z], where q = w + xi + yj + zk
    // fn set_rotation_quat_s2f(&mut self, rotation: [f32;4]);

    /// Set rotation matrix from S frame to F frame
    fn set_rotation_dcm_s2f(&mut self, rotation: [[f32;3];3]);

    /// Set both origin of S frame in F frame and rotation DCM from S to F
    fn set_mount(&mut self, origin: [f32;3], rotation: [[f32;3];3]);

    /// Initializes IMU
    /// 
    /// Specifically:
    /// - disables temperature sensor
    /// - Auto-select best clock source
    fn init(&mut self) -> Result<(), Self::Error>;
    
    /// Reads newest accelerometer data in G's as (x,y,z)
    /// 
    /// Assumes selected accelerometer full-scale range ACCEL_FS=0
    /// 
    /// # TODO
    /// Read the selected full-scale range and select the required scale factor
    fn read_accelerometer_g(&mut self) -> Result<(f32, f32, f32), Self::Error>;

    /// Reads newest accelerometer data in m/s^2 as (x,y,z)
    /// 
    /// Spec sheet specifies scale factors from raw to G
    /// Do this conversion first, then convert to m/s^2
    fn read_accelerometer_mps2(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    
    /// Helper function to convert accelerometer data from raw data to specified units
    /// 
    /// # Parameters
    /// - `raw_to_unit`: scaling factor to convert from raw to physical data
    fn read_accelerometer_units(&mut self,raw_to_units: f32) -> Result<(f32, f32, f32), Self::Error>;

    /// Reads newest accelerometer data
    /// 
    /// All accelerometer registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw accelerometer measurement as (x,y,z)
    fn read_accelerometer_raw(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    
    /// Reads newest gyroscope data in rad/s
    /// 
    /// Spec sheet specifies scale factors from raw to deg/sec
    /// Do this conversion first, then convert to rad/sec
    fn read_gyroscope_rps(&mut self) -> Result<(f32, f32, f32), Self::Error>;

    /// Reads newest gyroscope data in deg/s
    /// 
    /// Assumes selected gyroscope full-scale range GYRO_FS_SEL=0
    /// 
    /// # TODO
    /// Read the selected full-scale range and select the required scale factor
    fn read_gyroscope_dps(&mut self) -> Result<(f32, f32, f32), Self::Error>;

    /// Helper function to convert gyroscope data from raw data to specified units
    /// 
    /// # Parameters
    /// - `raw_to_unit`: scaling factor to convert from raw to physical data
    fn read_gyroscope_units(&mut self,raw_to_units: f32) -> Result<(f32, f32, f32), Self::Error>;

    /// Reads newest gyroscope data
    /// 
    /// All gyroscope registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw gyroscope measurement as (phi,theta,psi)
    /// 
    /// # TODO
    /// create function that reads all accel/gyro data in one write_read operation
    /// because all these registers are consecutive (temperature register as well)
    fn read_gyroscope_raw(&mut self) -> Result<(f32, f32, f32), Self::Error>;

    /// Estimate current attitude by integrating current gyroscope measurement over specified dt.
    /// 
    /// # Parameters
    /// - `attitude` (deg, deg, deg): estimate of attitude at last timestep as (phi, theta, psi)
    /// - `dt_us` (us): time elapsed since last attitude estimate
    /// 
    /// # Return
    /// `attitude` (deg, deg, deg): current attitude as (phi, theta, psi)
    fn attitude_from_direct_integration(&mut self, attitude: (f32,f32,f32), dt_us: u32) -> Result<(f32,f32,f32), Self::Error>;

    /// Estimate current velocity by integrating current accelerometer measurement over specified dt.
    /// 
    /// Assumes attitude is constant
    /// 
    /// # Parameters
    /// - `attitude` (deg, deg, deg): estimate of attitude at last timestep as (phi, theta, psi)
    /// - `dt_us` (us): time elapsed since last attitude estimate
    /// 
    /// # Return
    /// `attitude` (deg, deg, deg): current attitude as (phi, theta, psi)
    fn velocity_from_direct_integration(&mut self, velocity: (f32,f32,f32), dt_us: u32) -> Result<(f32,f32,f32), Self::Error>;
}

pub trait ImuWithMagnetometer : Imu {
    fn read_magnetometer_raw(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    
    /// Helper function to convert magnetometer data from raw data to specified units
    /// 
    /// # Parameters
    /// - `raw_to_units`: scaling factor to convert from raw to physical data
    fn read_magnetometer_units(&mut self,raw_to_units: f32) -> Result<(f32, f32, f32), Self::Error>;
}

pub trait ImuWithAdustableScale: Imu {

    /// Set sensitivity range of the accelerometer
    ///
    /// # Parameters
    /// - scale: can be 0, 1, 2, or 3. 
    ///     0: +- 2g
    ///     1: +- 4g
    ///     2: +- 8g
    ///     3: +- 16g
    fn set_accelerometer_scale(&mut self, scale: u8) -> Result<(), Self::Error>;
    
    /// Get sensitivity range of the accelerometer
    ///
    /// # Returns
    /// - scale: can be 0, 1, 2, or 3. 
    ///     0: +- 2g
    ///     1: +- 4g
    ///     2: +- 8g
    ///     3: +- 16g
    fn get_accelerometer_scale(&mut self) -> Result<u8, Self::Error>;
    
    /// Get sensitivity range of the gyroscope
    ///
    /// # Parameters
    /// - scale: can be 0, 1, 2, or 3. 
    ///     0: +- 250 dps
    ///     1: +- 500 dps
    ///     2: +- 1000 dps
    ///     3: +- 2000 dps
    fn set_gyroscope_scale(&mut self, scale: u8) -> Result<(), Self::Error>;
    
    /// Get sensitivity range of the gyroscope
    ///
    /// # Returns
    /// - scale: can be 0, 1, 2, or 3. 
    ///     0: +- 250 dps
    ///     1: +- 500 dps
    ///     2: +- 1000 dps
    ///     3: +- 2000 dps
    fn get_gyroscope_scale(&mut self) -> Result<u8, Self::Error>;
    
}

pub trait ImuWithRegisterBanks: Imu {

    /// Select register bank with which to interact
    /// 
    /// Should always be called before reading/writing to a register
    /// Argument should always be MY_REGISTER.get_bank() 
    /// because the register definitions track the associated bank
    /// # Example
    /// select_register_bank(ACCEL_CONFIG.get_bank())
    fn select_register_bank(&mut self, bank: u8) -> Result<(), Self::Error>;
}

#[derive(Copy, Clone)]
pub struct Measurement {
    accelerometer_s_mps2: (f32,f32,f32),
    gyroscope_s_rps: (f32,f32,f32),
}

impl Measurement {
    pub fn init() -> Self {
        Self {
            accelerometer_s_mps2: (0.0, 0.0, 0.0),
            gyroscope_s_rps: (0.0, 0.0, 0.0),
        }
    }

    pub fn update_accel_s_mps2(&mut self, accel: (f32,f32,f32)) {
        self.accelerometer_s_mps2 = accel;
    }
    pub fn update_gyroscope_s_rps(&mut self, gyro: (f32,f32,f32)) {
        self.gyroscope_s_rps = gyro;
    }

    pub fn get_accel_s_mps2(&self) -> [f32;3] {
        [
            self.accelerometer_s_mps2.0, 
            self.accelerometer_s_mps2.1, 
            self.accelerometer_s_mps2.2
        ]
    }
    pub fn get_accel_s_g(&self) -> [f32;3] {
        [
            self.accelerometer_s_mps2.0/EARTH_GRAVITY, 
            self.accelerometer_s_mps2.1/EARTH_GRAVITY, 
            self.accelerometer_s_mps2.2/EARTH_GRAVITY
        ]
    }

    pub fn get_gyro_s_rps(&self) -> [f32;3] {
        [
            self.gyroscope_s_rps.0, 
            self.gyroscope_s_rps.1, 
            self.gyroscope_s_rps.2]
    }

    pub fn get_gyro_s_dps(&self) -> [f32;3] {
        [
            self.gyroscope_s_rps.0*RAD_TO_DEG, 
            self.gyroscope_s_rps.1*RAD_TO_DEG, 
            self.gyroscope_s_rps.2*RAD_TO_DEG
        ]
    }

    /// Get last read data as a comma-separated string
    /// Ignores errors
    /// Units:
    /// Accel: m/s^2
    /// Gyro: rad/s
    pub fn report(&self) -> String<256> {
        use core::fmt::Write;
        let mut s: String<256> = String::new();
        write!(s, "{},{},{},{},{},{},", 
            self.accelerometer_s_mps2.0, self.accelerometer_s_mps2.1, self.accelerometer_s_mps2.2,
            self.gyroscope_s_rps.0, self.gyroscope_s_rps.1, self.gyroscope_s_rps.2).ok();
        s
    }
}