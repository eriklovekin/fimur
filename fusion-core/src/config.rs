use nalgebra::{
    Matrix3, 
    Vector3,
};

pub struct ImuComConfig {
    multiplexer_addr: (bool,bool,bool),
    multiplexer_bus: u8,
    sensor_addr: u8,
}

pub struct ImuAccelConfig {
    scale: u8,
}

pub struct ImuGyroConfig {
    scale: u8,
}
pub struct ImuPoseConfig {
    s2f: Matrix3<f32>,
    origin_f: Vector3::<f32>,
}

pub struct ImuConfig {
    communication:  ImuComConfig,
    accelerometer:  ImuAccelConfig,
    gyroscope:      ImuGyroConfig,
    pose:           ImuPoseConfig,
}

include!(concat!(env!("OUT_DIR"), "/sensor_configs.rs"));
