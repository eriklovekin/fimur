#![no_std]

use math::{
    wB_to_dquaternion,
};
use constants::{
    PI
};

const N_IMUS: usize = 2; // number of IMUs being used

pub struct Filter {
    dt_us: u32,
    sensors: [Imu; N_IMUS],
    /// All accelerometer measurements transformed into F frame
    last_accel_f_g:     [[f32;3];N_IMUS],
    /// All gyroscope measurements transformed into F frame
    last_gyro_f_dps:    [[f32;3];N_IMUS],
    /// Current estimate of accelerations in F frame [g]
    accel_est_f_g:      [[f32;3];1], 
    /// Current estimate of angular rates in F frame [deg/s]
    gyro_est_f_dps:     [[f32;3];1],
    /// Current estimate of attitude [deg]
    att_est_f_deg:        [[f32;3];1]
}

impl Filter {
    /// Complimentary filter from gyroscope and accelerometer measurements
    /// 
    /// Assumes F frame is not experiencing acceleration besides gravity.
    /// Accelerometer attitude formula from Ref 5
    /// 
    /// # Parameters
    /// - A: weight by which to favor gyroscope measurement. 0 <= A <= 1
    pub fn complimentary_filter_attitude(&mut self,A: f32) 
    -> [f32;3]
    {
        let gyro_vec_f:  [f32;N_IMUS] = self.transform_all_gyro_s2f();
        let accel_vec_f: [f32;N_IMUS] = self.transform_all_accel_s2f();

        let gyro_avg_f_dps: f32 = average_filter(gyro_vec_f);
        let accel_avg_f: f32 = average_filter(accel_vec_f);

        let att_gyro_f_deg: [f32;3] = self.att_est_f_d + gyro_avg_f*self.dt_us*1e6;
        let att_accel_f_rad: [f32;3] = [
                accel_avg_f[1].atan2(accel_avg_f[2]),
                (-accel_avg_f[0]).atan2(((accel_avg_f[1]*accel_avg_f[1]) + (accel_avg_f[2]*accel_avg_f[2])).sqrt()), 
                0
            ];
        let att_accel_f_deg: [f32;3] = att_accel_f_rad *180/PI;
        A*att_gyro_f_deg + (1-A)*att_accel_f_deg
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
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N];
        for (s,i) in sensors.iter().enumerate() {
            ret[i] = self.transform_imu_gyro_s2f(s);
        }
        last_gyro_f_dps = ret;
        ret
    }

    /// For each member sensor, transform accelerometer measurement into F frame
    pub fn transform_all_accel_s2f(&mut self) -> [[f32;3];N_IMUS] {
        let mut ret: [[f32;3];N_IMUS] = [[0.0; 3]; N];
        for (s,i) in sensors.iter().enumerate() {
            ret[i] = self.transform_imu_accel_s2f(s);
        }
        last_accel_f_g = ret;
        ret
    }
    
    /// For an Imu object, transform its gyroscope measurement into the F frame
    pub fn transform_imu_gyro_s2f(&mut self, imu: Imu) -> [f32;3] {
        self.transform_gyro_s2f(imu.meas().accelerometer_s_g,imu.rotation_dcm_s2f(), imu.origin_f,self.accel_est_f_g)
    }

    /// For an Imu object, transform its accelerometer measurement into the F frame
    pub fn transform_imu_accel_s2f(&mut self, imu: Imu) -> [f32;3] {
        self.transform_accel_s2f(imu.meas().accel,imu.rotation_dcm_s2f(),imu.origin_f,self.gyro_est_f_dps,self.accel_est_f_g)
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
    pub fn transform_accel_s2f(&mut self, accel_s: [f32;3], s2f: [[f32;3];3], origin_f: [[f32;3];3], w_f_dps: [[f32;3];3], a_f_dps2: [[f32;3];3]) -> [f32;3] {
        s2f*acces_s - cross(w_f_dps,cross(w_f_dps,origin_f)) - cross(a_f_dps2,origin_f)
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