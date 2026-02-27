#![no_std]

pub trait Imu {
    type Error;

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
    /// raw accelerometer measurement as (phi,theta,psi)
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
    fn attitude_from_direct_integration(&mut self, attitude: (f32,f32,f32), dt_us: u64) -> Result<(f32,f32,f32), Self::Error>;

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
    fn velocity_from_direct_integration(&mut self, velocity: (f32,f32,f32), dt_us: u64) -> Result<(f32,f32,f32), Self::Error>;
}

pub trait ImuWithMagnetometer : Imu {
    fn read_megnetometer(&mut self) -> Result<(f32, f32, f32), Self::Error>;
}

pub fn clear_bits(bits: u8, position: u8, size: u8) -> Result<u8, &'static str> {
    if position > 7 {
        return Err("position greater than size of bits");
    } 
    if size > position+1 {
        return Err("size must be <= position+1");
    } 
    if size == 0 {
        return Err("size must be > 0");
    }
    let mask: u8 = ((1<<size)-1) << position;
    Ok(bits & !mask)
}

pub trait ImuWithAdustableScale: Imu {
    fn set_accelerometer_scale(&mut self, scale: u8) -> Result<(), Self::Error>;
    fn get_accelerometer_scale(&mut self) -> Result<u8, Self::Error>;
    fn set_gyroscope_scale(&mut self, scale: u8) -> Result<(), Self::Error>;
    fn get_gyroscope_scale(&mut self) -> Result<u8, Self::Error>;
    
}

pub trait ImuWithRegisterBanks: Imu {
    fn select_register_bank(&mut self, bank: u8) -> Result<(), Self::Error>;
}