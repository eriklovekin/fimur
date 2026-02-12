#![no_std]

use embedded_hal::{
    i2c::I2c,
    delay
};
use imu_traits::{
    Imu,
    ImuWithAdjustableRange
};


pub struct Icm20948<I2C> {
    i2c: I2C,
    address: u8,
}

// impl<I2C, E> Imu for Icm20948<I2C>
// where 
//     I2C: I2c<Error = E>,
// {
    
// }

