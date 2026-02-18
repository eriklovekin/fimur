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

    /// Reads newest accelerometer data
    /// 
    /// All accelerometer registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw accelerometer measurement as (x,y,z)
    fn read_accelerometer(&mut self) -> Result<(f32,f32,f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address,&[ACCEL_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))
    }

    /// Reads newest gyroscope data
    /// 
    /// All gyroscope registers are consecutive, 
    /// so can be read with a single write_read operation
    /// 
    /// # Return
    /// raw accelerometer measurement as (x,y,z)
    /// 
    /// # TODO
    /// create function that reads all accel/gyro data in one write_read operation
    /// because all these registers are consecutive (temperature register as well)
    fn read_gyroscope(&mut self) -> Result<(f32, f32, f32), Self::Error> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address,&[GYRO_XOUT_H.addr],&mut buf)?;
        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let y = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let z = i16::from_be_bytes([buf[4], buf[5]]) as f32;
        Ok((x,y,z))    
    }
}

