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
pub fn wB_to_dquaternion(w:(f32, f32, f32), dt_us: u64) -> [f32;4] {
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
pub fn quaternion_to_euler_deg(q: [f32;4]) -> [f32;3] {
    roll    = atan2f(2*( q[2]*q[3] + q[0]*q[1] ), q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2) * 180/PI;
    pitch   =  -asin(2*( q[1]*q[3] - q[0]*q[2])) * 180/PI;
    yaw     = atan2f(2*( q[1]*q[2] + q[0]*q[3] ), q[0]^2 + q[1]^2 - q[2]^2 - q[3]^2) * 180/PI;
    (roll, pitch, yaw)
}

/// Convert from Direction Cosine Matrix (DCM) to Quaternion using Shepperd's algorithm
/// 
/// # Parameters
/// - dcm: DCM matrix
/// 
/// # Returns
/// - quaternion as [w, x, y, z]
pub fn dcm_to_quaternion_shepperd(dcm: [[f32;3];3]) -> [f32;4] {
    let c1 = dcm[0][0];
    let c2 = dcm[1][1];
    let c3 = dcm[2][2];
    let c4 = c1+c2+c3;
    let select: [f32;4] = [c1, c2, c3, c4];
    let max = select.max_by_key(|(_idx, &val)| val);
    
    let [q1, q2, q3, q4]: [f32;4] = [0.0;4];
    match max {
        0 => {
            q2 = (1 + c1 - c2 - c3).sqrt();
            q1 = ((dcm[2][1] - dcm[1][2]) / q2);
            q3 = ((dcm[0][1] + dcm[1][0]) / q2);
            q4 = ((dcm[2][0] + dcm[0][2]) / q2); },
        1 => {
            q3 = (1 - c1 + c2 - c3).sqrt();
            q1 = ((dcm[0][2] - dcm[2][0]) / q3);
            q2 = ((dcm[0][1] + dcm[1][0]) / q3);
            q4 = ((dcm[1][2] + dcm[2][1]) / q3); },
        2 => {
            q4 = (1 - c1 - c2 + c3).sqrt();
            q1 = ((dcm[1][0] - dcm[0][1]) / q4);
            q2 = ((dcm[2][0] + dcm[0][2]) / q4);
            q3 = ((dcm[2][1] + dcm[1][2]) / q4); },
        3 => {
            q1 = (1 + c1 + c2 + c3).sqrt();
            q2 = ((dcm[2][1] - dcm[1][2]) / q1);
            q3 = ((dcm[0][2] - dcm[2][0]) / q1);
            q4 = ((dcm[1][0] - dcm[0][1]) / q1); },
        _ => println("weird")
    }
    [q1, q2, q3, q4]
}

/// Convert from quaternion to Direction Cosine Matrix (DCM)
/// 
/// # Parameters
/// - q: quaternion as [w, x, y, z]
/// # Returns:
/// - DCM as [[f32;3];3]
pub fn quaternion_to_dcm(q: [f32;4]) -> [[f32;3];3] {
    let C11:f32 = q[0].pow(2) + q[1].pow(2) - q[2].pow(2) - q[3].pow(2);
    let C22:f32 = q[0].pow(2) - q[1].pow(2) + q[2].pow(2) - q[3].pow(2);
    let C11:f32 = q[0].pow(2) - q[1].pow(2) - q[2].pow(2) + q[3].pow(2);

    let C12:f32 = 2*(q[1]*q[2] - q[0]*q[3]);
    let C21:f32 = 2*(q[1]*q[2] + q[0]*q[3]);

    let C13:f32 = 2*(q[1]*q[3] + q[0]*q[2]);
    let C31:f32 = 2*(q[1]*q[3] - q[0]*q[2]);

    let C23:f32 = 2*(q[1]*q[3] - q[0]*q[1]);
    let C23:f32 = 2*(q[1]*q[3] + q[0]*q[1]);

    [[C11, C12, C13],
     [C21, C22, C23],
     [C31, C32, C33]]
}