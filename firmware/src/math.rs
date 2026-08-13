use core::f32::consts::PI;

use defmt::{
    panic
};

use libm::{
    sinf,
    cosf,
    atan2f,
    sqrtf,
    asinf,
};

/// Convert body angular rates to quaternion rates
/// 
/// Body angular rates are read from the IMU
/// # Parameters
/// - w: Body angular rates in (deg/s) as [x, y, z]
pub fn w_b_to_dquaternion(w:[f32;3], dt_us: u64) -> [f32;4] {
    let w_mag = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    let dt_s = dt_us as f32 / 1e6;
    let q1 = cosf(w_mag*dt_s/2.0);
    let q2 = (w[0]/w_mag)*sinf(w_mag*dt_s/2.0);
    let q3 = (w[1]/w_mag)*sinf(w_mag*dt_s/2.0);
    let q4 = (w[2]/w_mag)*sinf(w_mag*dt_s/2.0);
    [q1, q2, q3, q4]
}

/// Convert from quaternion to Euler angle
/// 
/// Formulas from Ref. 2
/// 
/// # Parameters
/// q: Quaternion
/// 
/// # Returns
/// - Euler angles (deg) as (roll, pitch, yaw)
pub fn quaternion_to_euler_deg(q: [f32;4]) -> [f32;3] {
    let roll: f32    = atan2f(2.*( q[2]*q[3] + q[0]*q[1] ), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*180.0/PI;
    let pitch: f32   =  asinf(-(2.*( q[1]*q[3] - q[0]*q[2])))*180.0/PI;
    let yaw: f32     = atan2f(2.*( q[1]*q[2] + q[0]*q[3] ), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])*180.0/PI;
    [roll, pitch, yaw]
}

/// Convert from Direction Cosine Matrix (DCM) to Quaternion using Shepperd's algorithm
/// 
/// Formula from Ref. 3
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
    let max = select.iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(idx, _)| idx)
        .unwrap();
    
    let [q1, q2, q3, q4]: [f32;4];
    match max {
        0 => {
            q2 = sqrtf(1.0 + c1 - c2 - c3);
            q1 = (dcm[2][1] - dcm[1][2]) / q2;
            q3 = (dcm[0][1] + dcm[1][0]) / q2;
            q4 = (dcm[2][0] + dcm[0][2]) / q2; },
        1 => {
            q3 = sqrtf(1.0 - c1 + c2 - c3);
            q1 = (dcm[0][2] - dcm[2][0]) / q3;
            q2 = (dcm[0][1] + dcm[1][0]) / q3;
            q4 = (dcm[1][2] + dcm[2][1]) / q3; },
        2 => {
            q4 = sqrtf(1.0 - c1 - c2 + c3);
            q1 = (dcm[1][0] - dcm[0][1]) / q4;
            q2 = (dcm[2][0] + dcm[0][2]) / q4;
            q3 = (dcm[2][1] + dcm[1][2]) / q4; },
        3 => {
            q1 = sqrtf(1.0 + c1 + c2 + c3);
            q2 = (dcm[2][1] - dcm[1][2]) / q1;
            q3 = (dcm[0][2] - dcm[2][0]) / q1;
            q4 = (dcm[1][0] - dcm[0][1]) / q1; },
        _ => panic!("Shepperd failed to find max value")
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
    let c11:f32 = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    let c22:f32 = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    let c33:f32 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    let c12:f32 = 2.0*(q[1]*q[2] - q[0]*q[3]);
    let c21:f32 = 2.0*(q[1]*q[2] + q[0]*q[3]);

    let c13:f32 = 2.0*(q[1]*q[3] + q[0]*q[2]);
    let c31:f32 = 2.0*(q[1]*q[3] - q[0]*q[2]);

    let c23:f32 = 2.0*(q[1]*q[3] - q[0]*q[1]);
    let c32:f32 = 2.0*(q[1]*q[3] + q[0]*q[1]);

    [[c11, c12, c13],
     [c21, c22, c23],
     [c31, c32, c33]]
}

/// cross product axb
pub fn cross3(a: [f32;3], b: [f32;3]) -> [f32;3] {
    [
        a[1]*b[2] - b[1]*a[2],
        a[0]*b[2] - b[0]*a[2],
        a[0]*b[1] - b[0]*a[1]
    ]
}

pub fn mat_vec_mult<const N: usize, const M: usize>(m: [[f32; N]; M], v: [f32; M]) -> [f32;N] {
    let mut result = [0.0;N];
    for i in 0..N {
        for j in 0..M {
            result[i] += m[i][j] * v[j];
        }
    }
    result
}

pub fn add_arrays<const N: usize>(a: [f32; N], b: [f32; N]) -> [f32;N] {
    let mut result = [0.0;N];
    for i in 0..N {
        result[i] = a[i] + b[i];
    }
    result
}