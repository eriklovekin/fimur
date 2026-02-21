#![no_std]

use embedded_hal::{
    i2c::I2c,
    // delay
};
use imu_traits::{
    Imu,
    // ImuWithAdjustableRange
};

mod registers;
use registers::bank0::*;
// use registers::bank1::*;
// use registers::bank2::*;
// use registers::bank3::*;

pub mod constants;

pub struct Icm20948<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Imu for Icm20948<I2C>
where 
    I2C: I2c<Error = E>,
{
    type Error = E;

    /// Initializes ICM20948
    /// 
    /// Specifically:
    /// - disables temperature sensor
    /// - Auto-select best clock source
    fn init(&mut self) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[PWR_MGMT_1.addr,0x09])?;
        // self.i2c.write(self.address,Accel); // Accelerometer settings
        Ok(())
    }

    /// Reads newest accelerometer data in m/s^2 as (x,y,z)
    /// 
    /// Spec sheet specifies scale factors from raw to G
    /// Do this conversion first, then convert to m/s^2
    fn read_accelerometer_mps2(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let gs = self.read_accelerometer_g()?;
        Ok((gs.0*constants::G_TO_MPS2, gs.1*constants::G_TO_MPS2, gs.2*constants::G_TO_MPS2))    
    }

    /// Reads newest accelerometer data in G's as (x,y,z)
    /// 
    /// Assumes selected accelerometer full-scale range ACCEL_FS=0
    /// 
    /// # TODO
    /// Read the selected full-scale range and select the required scale factor
    fn read_accelerometer_g(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        Ok(self.read_accelerometer_units(constants::ACCEL_FS0_SCALE)?)
    }

    /// Reads newest accelerometer data
    /// 
    /// All accelerometer registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw accelerometer measurement as (x,y,z)
    fn read_accelerometer_raw(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address,&[ACCEL_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))
    }

    /// Reads newest gyroscope data in deg/s
    /// 
    /// Assumes selected gyroscope full-scale range GYRO_FS_SEL=0
    /// 
    /// # TODO
    /// Read the selected full-scale range and select the required scale factor
    fn read_gyroscope_dps(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        Ok(self.read_gyroscope_units(constants::GYRO_FS0_SCALE)?)
    }

    /// Reads newest gyroscope data in rad/s
    /// 
    /// Spec sheet specifies scale factors from raw to deg/sec
    /// Do this conversion first, then convert to rad/sec
    fn read_gyroscope_rps(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let dps = self.read_gyroscope_dps()?;
        Ok((dps.0*constants::DEG_TO_RAD, dps.1*constants::G_TO_MPS2, dps.2*constants::G_TO_MPS2))
    }

    /// Reads newest gyroscope data
    /// 
    /// All gyroscope registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw accelerometer measurement as (phi,theta,psi)
    /// 
    /// # TODO
    /// create function that reads all accel/gyro data in one write_read operation
    /// because all these registers are consecutive (temperature register as well)
    fn read_gyroscope_raw(&mut self) -> Result<(f32, f32, f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address,&[GYRO_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))    
    }

    /// Helper function to convert accelerometer data from raw data to specified units
    /// 
    /// # Parameters
    /// - `raw_to_unit`: scaling factor to convert from raw to physical data
    fn read_accelerometer_units(&mut self, raw_to_units: f32) -> Result<(f32,f32,f32), Self::Error> {
        let raw = self.read_accelerometer_raw()?;
        Ok((raw.0*raw_to_units,raw.1*raw_to_units,raw.2*raw_to_units))
    }

    /// Helper function to convert gyroscope data from raw data to specified units
    /// 
    /// # Parameters
    /// - `raw_to_unit`: scaling factor to convert from raw to physical data
    fn read_gyroscope_units(&mut self, raw_to_units: f32) -> Result<(f32,f32,f32), Self::Error> {
        let raw = self.read_gyroscope_raw()?;
        Ok((raw.0*raw_to_units,raw.1*raw_to_units,raw.2*raw_to_units))
    }

    /// Estimate current attitude by integrating current gyroscope measurement over specified dt.
    /// 
    /// # Parameters
    /// - `attitude` (deg, deg, deg): estimate of attitude at last timestep as (phi, theta, psi)
    /// - `dt_us` (us): time elapsed since last attitude estimate
    /// 
    /// # Return
    /// `attitude` (deg, deg, deg): current attitude as (phi, theta, psi)
    fn attitude_from_direct_integration(&mut self, attitude: (f32,f32,f32), dt_us: u64) -> Result<(f32,f32,f32), Self::Error> {
        let (d_phi_dt, d_theta_dt, d_psi_dt) = self.read_gyroscope_dps()?;
        Ok((attitude.0 + d_phi_dt   *(dt_us as f32) / 1e6,
            attitude.1 + d_theta_dt *(dt_us as f32) / 1e6,
            attitude.2 + d_psi_dt   *(dt_us as f32) / 1e6))
    }
}


impl<I2C> Icm20948<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {i2c, address}
    }
}

