#![no_std]

use math::{
    wB_to_dquaternion,
};

const N_IMUS: usize = 2; // number of IMUs being used

pub struct Filter {
    dt_us: u32,
    sensors: [Imu; N_IMUS],
    /// All accelerometer measurements transformed into F frame
    last_accel_f_g:     [[f32;3];N_IMUS],
    /// All gyroscope measurements transformed into F frame
    last_gyro_f_dps:    [[f32;3];N_IMUS],
    /// Current estimate of accelerations along F frame axes [g]
    accel_est_f_g:      [[f32;3];1], 
    /// Current estimate of angular rates about F frame axes [deg/s]
    gyro_est_f_dps:     [[f32;3];1],
}

impl Filter {
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
    pub fn transform_all_gyro_s2f(&mut self, imu: Imu) -> [[f32;3];N_IMUS] {
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N];
        for (s,i) in sensors.iter().enumerate() {
            ret[i] = self.transform_imu_gyro_s2f(s);
        }
        last_gyro_f_dps = ret;
        ret
    }
    
    /// For an Imu object, transform its gyro measurement into the F frame
    pub fn transform_imu_gyro_s2f(&mut self, imu: Imu) -> [f32;3] {
        self.transform_gyro_s2f(imu.meas().gyroscope_s_dps,imu.rotation_dcm_s2f())
    }

    /// From a 3-axis gyroscope measurement in an S frame, calculate the measurement in the F frame
    /// 
    /// Assume S and F are rigidly fixed
    /// 
    /// # Parameters
    /// - gyro_s: gyroscope measurements in S frame (xS, yS, zS) [deg/s]
    /// - s2f: DCM rotating from S to F frame
    pub fn transform_gyro_s2f(&mut self, gyro_s: [f32;3], s2f: [[f32;3];3]) -> [f32;3] {
        s2f*gyro_s
    }

}