#![no_std]

pub trait Imu {
    type Error;

    fn init(&mut self) -> Result<(), Self::Error>;
    fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    fn read_gyroscope(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    fn attitude_from_direct_integration(&mut self, attitude: (f32,f32,f32), dt_us: u64) -> Result<(f32,f32,f32), Self::Error>;
}

pub trait ImuWithMagnetometer : Imu {
    fn read_megnetometer(&mut self) -> Result<(f32, f32, f32), Self::Error>;
}

pub trait ImuWithAdjustableRange :Imu {
    fn set_accelerometer_range(&mut self, range: i8) -> Result<(), Self::Error>;
    fn set_gyroscope_range(&mut self, range: i8) -> Result<(), Self::Error>;
}