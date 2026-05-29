use crate::math::{
    add_arrays,
    mat_vec_mult,
    cross3,
};

use libm::{
    atan2f,
    sqrtf,
};

use defmt::{
    info,
    panic
};

use imu_traits::{
    Imu,
};

use icm20948:: {
    Icm20948
};

use heapless::String;

use core::fmt::Write;
use imu_traits::constants::{
    PI
};

const N_IMUS: usize = 2; // number of IMUs being used
const REPORT_RAW_SIZE: usize = 20*N_IMUS;

pub struct Filter <I2C>{
    dt_us: u32,
    sensors: [Icm20948<I2C>; N_IMUS],
    /// All accelerometer measurements transformed into F frame
    last_accel_f_g:     [[f32;3];N_IMUS],
    /// All gyroscope measurements transformed into F frame
    last_gyro_f_dps:    [[f32;3];N_IMUS],
    /// Current estimate of accelerations in F frame [g]
    accel_est_f_g:      [f32;3], 
    /// Current estimate of angular rates in F frame [deg/s]
    gyro_est_f_dps:     [f32;3],
    /// Current estimate of attitude [deg]
    att_est_f_deg:      [f32;3]
}

impl<I2C: embedded_hal::i2c::I2c> Filter <I2C>{
    pub fn new(dt_us:u32, s: [Icm20948<I2C>; N_IMUS]) -> Self {
        Self {
            dt_us: dt_us,
            sensors: s,
            last_accel_f_g: [[0.0;3];N_IMUS],
            last_gyro_f_dps:  [[0.0;3];N_IMUS],
            accel_est_f_g:  [0.0;3],
            gyro_est_f_dps: [0.0;3],
            att_est_f_deg:  [0.0;3],
        }
    }

    pub fn init(&mut self) {
        for (i,si) in self.sensors.iter_mut().enumerate() {
            match si.init() {
                Ok(_) => { info!("imu {} init succeeded!",i);}
                Err(e) => {
                    info!("imu {} init failed: {}", i, defmt::Debug2Format(&e));
                    panic!("Manual panic after init failure");
                }
            }
        }
    }

    pub fn sensor(&mut self, i:usize) -> &mut Icm20948<I2C> {
        if i >= N_IMUS {
            panic!("attemped to access invalid sensor. index must be < {}", N_IMUS);
        }
        &mut self.sensors[i]
    }

    /// Complimentary filter from gyroscope and accelerometer measurements
    /// 
    /// Assumes F frame is not experiencing acceleration besides gravity.
    /// Accelerometer attitude formula from Ref 5
    /// 
    /// # Parameters
    /// - a: weight by which to favor gyroscope measurement. 0 <= a <= 1
    pub fn complimentary_filter_attitude(&mut self,a: f32) 
    -> [f32;3]
    {
        let gyro_vec_f:  [[f32; 3];N_IMUS] = self.transform_all_gyro_s2f();
        let accel_vec_f: [[f32; 3];N_IMUS] = self.transform_all_accel_s2f();

        let gyro_avg_f_dps: [f32;3] = self.average_filter(gyro_vec_f);
        let accel_avg_f: [f32;3] = self.average_filter(accel_vec_f);

        let att_gyro_f_deg: [f32;3] = add_arrays(
            self.att_est_f_deg, 
            gyro_avg_f_dps.map(|x| x*(self.dt_us as f32) *1.0e6)
        );
        let att_accel_f_rad: [f32;3] = [
                atan2f(accel_avg_f[1],accel_avg_f[2]),
                atan2f(-accel_avg_f[0],sqrtf((accel_avg_f[1]*accel_avg_f[1]) + (accel_avg_f[2]*accel_avg_f[2]))), 
                0.0
            ];
        let att_accel_f_deg: [f32;3] = att_accel_f_rad.map(|x| x*180.0/PI);
        add_arrays(att_gyro_f_deg.map(|x| x*a),att_accel_f_deg.map(|x| x*(1.0-a)))
    }

    /// Average of M N-axis sensors
    /// # Parameters
    /// - ar: array containing all sensor measurements
    pub fn average_filter<const N_AXES: usize, const M_SENSORS: usize>(&mut self, ar: [[f32;N_AXES];M_SENSORS]) -> [f32;N_AXES] {
        let ret = self.weighted_average_filter(ar,[1.0/M_SENSORS as f32;M_SENSORS]);
        ret
    }

    /// Weighted average of M N-axis sensors with weight for sensor i given by w[i]
    /// 
    /// Normalizes by sum(w) in case it is not 1.0
    /// 
    /// # Parameters
    /// - ar: array containing all sensor measurements
    /// - w: array of weights (1 per sensor)
    pub fn weighted_average_filter<const N_AXES: usize, const M_SENSORS: usize>(&mut self, ar: [[f32;N_AXES];M_SENSORS], w: [f32;M_SENSORS]) -> [f32;N_AXES] {
        let mut ret: [f32;N_AXES] = [0.0;N_AXES];
        let w_sum: f32 = w.iter().sum(); /* in case total != 1 */
        for i in 0..N_AXES {
            for j in 0..M_SENSORS {
                ret[i] += w[j]*ar[i][j];
            }
            ret[i] /= w_sum;
        }
        ret
    }

    /// For each member sensor, transform gyro measurement into F frame
    pub fn transform_all_gyro_s2f(&mut self) -> [[f32;3];N_IMUS] {
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N_IMUS];
        for (i,s) in self.sensors.iter().enumerate() {
            ret[i] = self.transform_imu_gyro_s2f(s);
        }
        self.last_gyro_f_dps = ret;
        ret
    }

    /// For each member sensor, transform accelerometer measurement into F frame
    pub fn transform_all_accel_s2f(&mut self) -> [[f32;3];N_IMUS] {
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N_IMUS];
        for (i,s) in self.sensors.iter().enumerate() {
            ret[i] = self.transform_imu_accel_s2f(s);
        }
        self.last_accel_f_g = ret;
        ret
    }
    
    /// For an Imu object, transform its gyroscope measurement into the F frame
    pub fn transform_imu_gyro_s2f(&self, imu: &Icm20948<I2C>) -> [f32;3] {
        self.transform_gyro_s2f(imu.meas().get_gyro_s_dps(),imu.rotation_dcm_s2f())
    }

    /// For an Imu object, transform its accelerometer measurement into the F frame
    pub fn transform_imu_accel_s2f(&self, imu: &Icm20948<I2C>) -> [f32;3] {
        self.transform_accel_s2f(imu.meas().get_accel_s_g(),imu.rotation_dcm_s2f(),imu.origin_f(),self.gyro_est_f_dps,self.accel_est_f_g)
    }

    /// From a 3-axis accelerometer measurement in an S frame, calculate the measurement in the F frame
    /// 
    /// Assume S and F are rigidly fixed.
    /// Eq: a_f = s2f*a_s - (w x (w x r_s)) - (alpha x r)
    /// 
    /// # Parameters
    /// - accel_s: accelerometer measurements in S frame (xS, yS, zS) [deg/s]
    /// - s2f: DCM rotating from S to F frame
    /// - origin_f: origin of S frame in F frame
    /// - w_f_dps: rotational rate of body in F frame [deg/s]
    /// - a_f_dps2: rotational acceleration of body in F frame [deg/s^2]
    pub fn transform_accel_s2f(&self, accel_s: [f32;3], s2f: [[f32;3];3], origin_f: [f32;3], w_f_dps: [f32;3], a_f_dps2: [f32;3]) -> [f32;3] {
        add_arrays(
            add_arrays(
                mat_vec_mult(s2f,accel_s), 
                cross3(w_f_dps,cross3(w_f_dps,origin_f)).map(|x| -x)), 
            cross3(a_f_dps2,origin_f).map(|x| -x))
    }

    /// From a 3-axis gyroscope measurement in an S frame, calculate the measurement in the F frame
    /// 
    /// Assume S and F are rigidly fixed
    /// 
    /// # Parameters
    /// - gyro_s: gyroscope measurements in S frame (xS, yS, zS) [deg/s]
    /// - s2f: DCM rotating from S to F frame
    pub fn transform_gyro_s2f(&self, gyro_s: [f32;3], s2f: [[f32;3];3]) -> [f32;3] {
        mat_vec_mult(s2f,gyro_s)
    }

    /// Read accelerometer and gyroscope measurements from all sensors
    /// 
    /// Take them all sequentially, assume timing difference is negligible
    /// 
    /// # TODO
    /// Use `?` to pass error messages through. Requires shared error type for all sensors
    pub fn read_all(&mut self){
        for (i,si) in self.sensors.iter_mut().enumerate() {
            si.read_accelerometer_mps2()
                .unwrap_or_else(|_| panic!("failed to read accelerometer of sensor {}", i));
            si.read_gyroscope_dps()
                .unwrap_or_else(|_| panic!("failed to read gyroscope of sensor {}", i));
        }
    }

    // Return number of sensors in filter object
    pub fn get_n_sensors(&self) -> usize {
        N_IMUS as usize
    }

    /// Get last read data from all sensors as a comma-separated string
    /// Groups measurements by sensor not by type
    /// Ignores errors
    pub fn report_raw(&self) -> String<REPORT_RAW_SIZE> {
        let mut s: String<REPORT_RAW_SIZE> = String::new();
        for si in self.sensors.iter() {
            write!(s,"{}",si.meas().report()).ok();
        }
        s
    }

}