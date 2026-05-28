#![no_std]

use embedded_hal::{
    i2c::I2c,
    // delay
};
use imu_traits::{ImuWithAdustableScale, ImuWithRegisterBanks};
use imu_traits::{
    Imu,
    Measurement,
    constants
};

mod registers;
use registers::bank0::*;
// use registers::bank1::*;
use registers::bank2::*;

use crate::registers::{BITSHIFT_REG_SELECT, BITSHIFT_SCALE};
// use registers::bank3::*;

pub mod error;

pub struct Icm20948<I2C> {
    i2c: I2C,
    address: u8,
    /// position of S origin of IMU in F frame
    origin_f: [f32;3], 
    /// Direction Cosine Matrix rotating from S to F frame
    rotation_dcm_s2f: [[f32; 3];3],
    // measured data
    meas: Measurement,
}

impl<I2C> Icm20948<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c, 
            address, 
            origin_f: [0.0;3], 
            rotation_dcm_s2f: [[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0],
                               [0.0, 0.0, 1.0]],
            meas: Measurement::init()
        }
    }

    pub fn new_with_mount(i2c: I2C, address: u8, origin_f: [f32; 3], rotation_s2f: [[f32; 3];3]) -> Self {
        Self {
            i2c, 
            address, 
            origin_f, 
            rotation_dcm_s2f: rotation_s2f,
            meas: Measurement::init()
        }
    }
}

impl<I2C, E> Imu for Icm20948<I2C>
where 
    I2C: I2c<Error = E>,
{
    type Error = error::ImuError<E>;

    fn origin_f(&self) -> [f32;3] {
        self.origin_f
    }

    fn rotation_dcm_s2f(&self) -> [[f32;3];3] {
        self.rotation_dcm_s2f
    }

    fn meas(&self) -> Measurement {
        self.meas
    }

    fn set_origin_f(&mut self, origin: [f32;3]) {
        self.origin_f = origin;
    }

    fn set_rotation_dcm_s2f(&mut self, rotation: [[f32;3];3]) {
        self.rotation_dcm_s2f = rotation;
    }

    fn set_mount(&mut self, origin: [f32;3], rotation: [[f32;3];3]) {
        self.set_origin_f(origin);
        self.set_rotation_dcm_s2f(rotation);
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.select_register_bank(PWR_MGMT_1.get_bank())?;
        // Autoselect best clock source and disable temperature sensor
        self.i2c.write(self.address, &[PWR_MGMT_1.addr,0x09])?;
        Ok(())
    }

    fn read_accelerometer_mps2(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let gs = self.read_accelerometer_g()?;
        Ok((gs.0*constants::G_TO_MPS2, gs.1*constants::G_TO_MPS2, gs.2*constants::G_TO_MPS2))    
    }

    fn read_accelerometer_g(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let gs = self.read_accelerometer_units(constants::ACCEL_FS0_SCALE)?;
        self.meas.update_accel_s_g(gs);
        Ok(gs)
    }

    fn read_accelerometer_raw(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.select_register_bank(ACCEL_XOUT_H.get_bank())?;
        self.i2c.write_read(self.address,&[ACCEL_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))
    }

    fn read_gyroscope_dps(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let dps = self.read_gyroscope_units(constants::GYRO_FS0_SCALE)?;
        self.meas.update_gyroscope_s_dps(dps);
        Ok(dps)
    }

    fn read_gyroscope_rps(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let dps = self.read_gyroscope_dps()?;
        Ok((dps.0*constants::DEG_TO_RAD, dps.1*constants::G_TO_MPS2, dps.2*constants::G_TO_MPS2))
    }

    fn read_gyroscope_raw(&mut self) -> Result<(f32, f32, f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.select_register_bank(GYRO_XOUT_H.get_bank())?;
        self.i2c.write_read(self.address,&[GYRO_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))    
    }

    fn read_accelerometer_units(&mut self, raw_to_units: f32) -> Result<(f32,f32,f32), Self::Error> {
        let raw = self.read_accelerometer_raw()?;
        Ok((raw.0*raw_to_units,raw.1*raw_to_units,raw.2*raw_to_units))
    }

    fn read_gyroscope_units(&mut self, raw_to_units: f32) -> Result<(f32,f32,f32), Self::Error> {
        let raw = self.read_gyroscope_raw()?;
        Ok((raw.0*raw_to_units,raw.1*raw_to_units,raw.2*raw_to_units))
    }

    fn attitude_from_direct_integration(&mut self, attitude: (f32,f32,f32), dt_us: u32) -> Result<(f32,f32,f32), Self::Error> {
        let (d_phi_dt, d_theta_dt, d_psi_dt) = self.read_gyroscope_dps()?;
        // Ok((attitude.0 + (d_phi_dt-9.5)   * (dt_us as f32)/1e6,
        //     attitude.1 + (d_theta_dt-4.5) * (dt_us as f32)/1e6,
        //     attitude.2 + (d_psi_dt-0.5)   * (dt_us as f32)/1e6))
        Ok((attitude.0 + (d_phi_dt)   * (dt_us as f32)/1e6,
            attitude.1 + (d_theta_dt) * (dt_us as f32)/1e6,
            attitude.2 + (d_psi_dt)   * (dt_us as f32)/1e6))
    }

    fn velocity_from_direct_integration(&mut self, velocity: (f32,f32,f32), dt_us: u32) -> Result<(f32,f32,f32), Self::Error> {
        let (ax, ay, az) = self.read_accelerometer_mps2()?;
        Ok((velocity.0 + (ax) * (dt_us as f32)/1e6,
            velocity.1 + (ay) * (dt_us as f32)/1e6,
            velocity.2 + (az) * (dt_us as f32)/1e6))
    }
}

impl<I2C, E> ImuWithAdustableScale for Icm20948<I2C>
where 
    I2C: I2c<Error = E>,
{
    fn set_accelerometer_scale(&mut self, scale: u8) -> Result<(), Self::Error> {
        let mut config= [0u8];
        self.select_register_bank(ACCEL_CONFIG.get_bank())?;
        self.i2c.write_read(self.address,&[ACCEL_CONFIG.addr],&mut config)?;
        config[0] &= !0b0000_0110;
        config[0] |= (scale << BITSHIFT_SCALE) & 0b0000_0110;
        
        self.i2c.write(self.address,&[ACCEL_CONFIG.addr,config[0]])?;
        Ok(())
    }

    fn get_accelerometer_scale(&mut self) -> Result<u8, Self::Error> {
        let mut config= [0u8];
        self.select_register_bank(ACCEL_CONFIG.get_bank())?;
        self.i2c.write_read(self.address,&[ACCEL_CONFIG.addr],&mut config)?;
        let scale = (config[0] & 0b0000_0110) >> BITSHIFT_SCALE;
        Ok(scale)
    }

    fn set_gyroscope_scale(&mut self, scale: u8) -> Result<(), Self::Error> {
        if scale > 3 {
            return Err(error::ImuError::InvalidSetGyroscopeScale)
        }
        let mut config= [0u8];
        self.select_register_bank(GYRO_CONFIG_1.get_bank())?;
        self.i2c.write_read(self.address,&[GYRO_CONFIG_1.addr],&mut config)?;
        config[0] &= !0b0000_0110;
        config[0] |= (scale << BITSHIFT_SCALE) & 0b0000_0110;
        
        self.i2c.write(self.address,&[GYRO_CONFIG_1.addr,config[0]])?;
        Ok(())
    }

    fn get_gyroscope_scale(&mut self) -> Result<u8, Self::Error> {
        let mut config= [0u8];
        self.select_register_bank(GYRO_CONFIG_1.get_bank())?;
        self.i2c.write_read(self.address,&[GYRO_CONFIG_1.addr],&mut config)?;
        let scale = (config[0] & 0b0000_0110) >> BITSHIFT_SCALE;
        Ok(scale)
    }
}

impl<I2C, E> ImuWithRegisterBanks for Icm20948<I2C>
where 
    I2C: I2c<Error = E>,
{
    fn select_register_bank(&mut self, bank: u8) -> Result<(), Self::Error> {
        if bank > 3 {
            return Err(error::ImuError::InvalidRegisterBank)
        }
        self.i2c.write(self.address,&[REG_BANK_SEL.addr,bank<<BITSHIFT_REG_SELECT])?;
        Ok(())
    }
}