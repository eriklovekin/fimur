use crate::math::{
    add_arrays,
    mat_vec_mult,
    cross3,
};

use nalgebra::{
    SMatrix,
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

const N_IMUS: usize = 4; // number of IMUs being used
/// two sig figs and a comma
const F32_SIZE: usize = 5;
/// six floats and six commas for each IMU
const REPORT_RAW_SIZE: usize = 6*F32_SIZE*N_IMUS; 
/// three floats for each set of 3DoF (rotational, translational)
const REPORT_EST_3DOF_SIZE: usize = 3*F32_SIZE;
// /// size of report of current estimated pose
// const REPORT_EST_POSE_SIZE: usize = 2*REPORT_EST_3DOF_SIZE;
/// size of full report of current estimated state
const REPORT_EST_STATE_SIZE: usize = 4*REPORT_EST_3DOF_SIZE;

pub struct Filter <I2C>{
    dt_us: u32,
    sensors: [Icm20948<I2C>; N_IMUS],
    /// All accelerometer measurements transformed into F frame
    last_accel_f_g:     [[f32;3];N_IMUS],
    /// All gyroscope measurements transformed into F frame
    last_gyro_f_dps:    [[f32;3];N_IMUS],
    /// Current estimate of accelerations in F frame [g]
    accel_est_f_g:      [f32;3], 
    /// Current estimate of velocity in L frame [m/s]
    v_est_l_mps:        [f32;3],
    /// Current estimate of position in L frame [m]
    r_est_l_m:          [f32;3],
    /// Current estimate of angular rates in F frame [deg/s]
    w_est_f_dps:        [f32;3],
    /// Current estimate of attitude [deg]
    att_est_f_deg:      [f32;3],
    /// State
    state:      SMatrix<f32,12,12>,
    /// Covariance Matrix
    covariance: SMatrix<f32,12,12>,
    /// State transition matrix
    state_transition_matrix: SMatrix<f32,12,12>,
    process_noise_covariance: SMatrix<f32,12,12>,
}

impl<I2C: embedded_hal::i2c::I2c> Filter <I2C>{
    pub fn new(dt_us:u32, s: [Icm20948<I2C>; N_IMUS]) -> Self {
        info!("imu mem size: {}",core::mem::size_of::<Icm20948<I2C>>());
        info!("filter mem size: {}",core::mem::size_of::<Filter<I2C>>());
        Self {
            dt_us: dt_us,
            sensors: s,
            last_accel_f_g:   [[0.0;3];N_IMUS],
            last_gyro_f_dps:  [[0.0;3];N_IMUS],
            accel_est_f_g:  [0.0;3],
            v_est_l_mps:    [0.0;3],
            r_est_l_m:      [0.0;3],
            w_est_f_dps:    [0.0;3],
            att_est_f_deg:  [0.0;3],
            state:          SMatrix::zeros(),
            covariance:     SMatrix::identity(), // placeholder until more accurate P0 is implemented
            state_transition_matrix:    SMatrix::zeros(),
            process_noise_covariance:   SMatrix::zeros(),
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

    pub fn init_default_kinematics(&mut self) {
        let dt = (self.dt_us as f32) /1.0e6;
        let a = [
            [1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt],

            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        self.state_transition_matrix = SMatrix::from_fn(|i,j| a[i][j]);
    }

    /// Construct a process noise covariance matrix that assumes 
    /// translation and rotation are decoupled.
    pub fn init_block_process_noise_covariance(&mut self) {
        let dt = (self.dt_us as f32) /1.0e6;
        // assume process noise is isotropic
        let std_accel: f32 = 1.0; // standard deviation of process acceleration noise; m/s^2
        let std_alpha: f32 = 0.1; // standard deviation of process angular acceleration noise; g
        let mut q = SMatrix::<f32,12,12>::zeros();
        let dt2 = dt * dt;
        let dt3 = dt * dt * dt;
        let dt4 = dt * dt * dt * dt;
        
        let q_block =  [
            [dt4/4.0,       0.0,        0.0,    dt3/2.0,        0.0,        0.0],
            [    0.0,   dt4/4.0,        0.0,        0.0,    dt3/2.0,        0.0],
            [    0.0,       0.0,    dt4/4.0,        0.0,        0.0,    dt3/2.0],
            
            [dt3/2.0,        0.0,        0.0,       dt2, 0.0, 0.0],
            [    0.0,    dt3/2.0,        0.0,       0.0, dt2, 0.0],
            [    0.0,        0.0,    dt3/2.0,       0.0, 0.0, dt2],
        ];
        let q_block_trans = 
            SMatrix::<f32,6,6>::from_fn(|i,j| 
                std_accel*std_accel*q_block[i][j]);
        let q_block_rot   = 
            SMatrix::<f32,6,6>::from_fn(|i,j| 
                std_alpha*std_alpha*q_block[i][j]);
        for i in 0..12 {
            for j in 0..12 {
                if i < 6 && j < 6 {
                    q[(i,j)] = q_block_trans[(i,j)];
                } else if i >= 6 && j >= 6 {
                    q[(i,j)] = q_block_rot[(i,j)];

                }
            }
        }
        self.process_noise_covariance = SMatrix::from_fn(|i,j| q[(i,j)]);
    }

    /// Kalman Filter predict step:
    /// x_k = A*X_k-1 + B*u_k
    /// P_k = A*P_k-1*A_T + Q
    pub fn kalman_predict(&mut self) {
        self.state = self.state_transition_matrix * self.state;
        // Assuming no proccess noise for now
        self.covariance = self.state_transition_matrix * self.covariance * self.state_transition_matrix.transpose()
                            + self.process_noise_covariance;
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

    /// Intuitive Filter
    pub fn intuitive_filter(&mut self) {
        // Step 1: Read new data
        self.read_all();

        // Step 2: Transform angular rates to F frame
        self.transform_all_gyro_s2f();

        // Step 3: Find mean of angular rates. Treat this as truth
        self.w_est_f_dps = self.average_filter(self.last_gyro_f_dps);

        // Step 4: Integrate to get attitude
        self.att_est_f_deg = self.w_est_f_dps.map(|x| x*(self.dt_us as f32)*1.0e-6);

        // Step 5: transform measured accelerations to F frame
        self.transform_all_accel_s2f();

        // Step 6: find mean of accelerations. Treat this as truth
        self.accel_est_f_g = self.average_filter(self.last_accel_f_g);

        // Step 7: transform F accelerations into L frame
        // let acc_est_l_g = math::

        // Step 8: Integrate L accelerations to find velocity
        // self.v_est_l_mps = 

        // Step 9: Integrate L velocities to find position

    }

    /// Assume that:
    /// - IMUs are colocated and aligned
    /// - Identical sensors
    /// # Return
    /// average of all measurements from all sensors
    pub fn most_naive_filter_possible(&mut self) {
        let mut state = [[0.0f32;6];N_IMUS];
        for (i, si) in self.sensors.iter().enumerate() {
            let accel = si.meas().get_accel_s_g();
            let gyro = si.meas().get_gyro_s_dps();
            for j in 0..accel.len() {
                state[i][j] = accel[j];
                state[i][j+3] = gyro[j];
            }
        }
        let avg_state = self.average_filter(state);
        self.accel_est_f_g = [avg_state[0],avg_state[1],avg_state[2]];
        self.w_est_f_dps = [avg_state[3],avg_state[4],avg_state[5]];
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
        self.transform_accel_s2f(imu.meas().get_accel_s_g(),imu.rotation_dcm_s2f(),imu.origin_f(),self.w_est_f_dps,self.accel_est_f_g)
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

    /// Return number of sensors in filter object
    pub fn get_n_sensors(&self) -> usize {
        N_IMUS as usize
    }

    /// Get last read data from all sensors as a comma-separated string
    /// Groups measurements by sensor not by type
    /// ex. "accel1_x,accel1_y,accel1_z,gyro1_x,gyro1_y,gyro1_z,accel2_x,..."
    /// Ignores errors
    pub fn report_raw(&self) -> String<512> {
        let mut s: String<512> = String::new();
        for si in self.sensors.iter() {
            write!(s,"{}",si.meas().report()).ok();
        }
        s
    }

    /// Get the last virtual measurement of the filter
    /// accel_x, accel_y, accel_z, gyro_x, gyro_y, gyroz
    /// Ignores errors
    pub fn report_virtual_meas(&self) -> String<{2*REPORT_EST_3DOF_SIZE}> {
        let mut s: String<{2*REPORT_EST_3DOF_SIZE}> = String::new();
        write!(s,"{}{}",
            self.report_est_accel(),
            self.report_est_w(),
        ).ok();
        s
    }

    /// Get last estimated acceleration as a comma-separated string
    /// Groups measurements by sensor not by type
    /// Ignores errors
    pub fn report_est_accel(&self) -> String<REPORT_EST_3DOF_SIZE> {
        let mut s: String<REPORT_EST_3DOF_SIZE> = String::new();
        write!(s,"{},{},{},",
            self.accel_est_f_g[0],
            self.accel_est_f_g[1],
            self.accel_est_f_g[2]
        ).ok();
        s
    }

    /// Get last velocity estimate in L frame (m/s)
    pub fn report_est_velocity(&self) -> String<REPORT_EST_3DOF_SIZE> {
        let mut s: String<REPORT_EST_3DOF_SIZE> = String::new();
        write!(s,"{},{},{},",
            self.v_est_l_mps[0],
            self.v_est_l_mps[1],
            self.v_est_l_mps[2]
        ).ok();
        s
    }

    /// Get last position estimate in L frame (m)
    pub fn report_est_position(&self) -> String<REPORT_EST_3DOF_SIZE> {
        let mut s: String<REPORT_EST_3DOF_SIZE> = String::new();
        write!(s,"{},{},{},",
            self.r_est_l_m[0],
            self.r_est_l_m[1],
            self.r_est_l_m[2]
        ).ok();
        s
    }

    /// Get last angular rate estimate in F frame (deg/s)
    pub fn report_est_w(&self) -> String<REPORT_EST_3DOF_SIZE> {
        let mut s: String<REPORT_EST_3DOF_SIZE> = String::new();
        write!(s,"{},{},{},",
            self.w_est_f_dps[0],
            self.w_est_f_dps[1],
            self.w_est_f_dps[2]
        ).ok();
        s
    }

    /// Get last attitude estimate in F frame (deg)
    pub fn report_est_attitude(&self) -> String<REPORT_EST_3DOF_SIZE> {
        let mut s: String<REPORT_EST_3DOF_SIZE> = String::new();
        write!(s,"{},{},{},",
            self.att_est_f_deg[0],
            self.att_est_f_deg[1],
            self.att_est_f_deg[2]
        ).ok();
        s
    }

    /// Get last estimate of state as \[re,rn,ru,ve,vn,vu,thetax,thetay,thetaz,wx,wy,xz\] (m,m/s,deg,deg/s)
    pub fn report_est_state(&self) -> String<REPORT_EST_STATE_SIZE> {
        let mut s: String<REPORT_EST_STATE_SIZE> = String::new();
        write!(s,"{}{}{}{}",
            self.report_est_position(),
            self.report_est_velocity(),
            self.report_est_attitude(),
            self.report_est_w(),
        ).ok();
        s
    }

}