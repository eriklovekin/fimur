use nalgebra::{
    Matrix3, 
    Vector3,
};

pub struct ImuComConfig {
    pub multiplexer_addr: (bool,bool,bool),
    pub multiplexer_bus: u8,
    pub sensor_addr: u8,
}

pub struct ImuAccelConfig {
    pub scale: u8,
}

pub struct ImuGyroConfig {
    pub scale: u8,
}
pub struct ImuPoseConfig {
    pub s2f: Matrix3<f32>,
    pub origin_f: Vector3::<f32>,
}

pub struct ImuConfig {
    pub communication:  ImuComConfig,
    pub accelerometer:  ImuAccelConfig,
    pub gyroscope:      ImuGyroConfig,
    pub pose:           ImuPoseConfig,
}

include!(concat!(env!("OUT_DIR"), "/sensor_configs.rs"));
