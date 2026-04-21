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
    /// For each sensor in the struct, transform gyro measurement into F frame
    pub fn transform_all_gyro_s2f(&mut self, imu: Imu) -> [[f32;3];N_IMUS] {
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N];
        for (s,i) in sensors.iter().enumerate() {
            ret[i] = transform_imu_gyro_s2f(s);
        }
        last_gyro_f_dps = ret
    }
    

    /// For an Imu object, transform its gyro measurement into the F frame
    pub fn transform_imu_gyro_s2f(&mut self, imu: Imu) -> [f32;3] {
        transform_gyro_s2f(imu.meas().gyroscope_s_dps,imu.rotation_dcm_s2f())
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