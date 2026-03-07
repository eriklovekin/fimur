#![no_std]

use libm::{
    sinf,
    cosf,
    atan2f,
    sqrtf
};
use core::f32::consts::PI;


/// Convert body angular rates to quaternion rates
/// 
/// Body angular rates are read from the IMU
/// # Parameters
/// - w: Body angular rates in (deg/s) as [x, y, z]
pub fn wB_to_dquaternion(w:(f32, f32, f32), dt_us: u64) -> (f32, f32, f32, f32) {
    let w_mag = sqrtf(w.0*w.0 + w.1*w.1 + w.2*w.2);
    let dt_s = dt_us as f32 / 1e6;
    let q1 = cosf(w_mag*dt_s/2.);
    let q2 = (w.0/w_mag)*sinf(w_mag*dt_s/2.);
    let q3 = (w.1/w_mag)*sinf(w_mag*dt_s/2.);
    let q4 = (w.2/w_mag)*sinf(w_mag*dt_s/2.);
    (q1, q2, q3, q4)
}

/// Convert from quaternion to Euler angle
/// 
/// Formulas from https://mwrona.com/posts/attitude-ekf/
/// 
/// # Parameters
/// q: Quaternion
/// 
/// # Returns
/// - Euler angles (deg) as (roll, pitch, yaw)
pub fn quaternion_to_euler_deg(q: (f32, f32, f32, f32)) -> (f32, f32, f32) {
    roll    = atan2f(2*( q[2]*q[3] + q[0]*q[1] ), q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2) * 180/PI;
    pitch   =  -asin(2*( q[1]*q[3] - q[0]*q[2])) * 180/PI;
    yaw     = atan2f(2*( q[1]*q[2] + q[0]*q[3] ), q[0]^2 + q[1]^2 - q[2]^2 - q[3]^2) * 180/PI;
    (roll, pitch, yaw)
}