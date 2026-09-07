use serde::{
    Deserialize,
};

use nalgebra::{
    Matrix3, 
    Vector3,
};

use std::fs;
use std::env;
use std::path::Path;

#[derive(serde::Deserialize)]
struct Manifest {
    sensors: Vec<String>,
}

#[derive(Deserialize)]
struct ImuComConfigRaw {
    multiplexer_addr: (bool,bool,bool),
    multiplexer_bus: u8,
    sensor_addr: u8,
}

fn format_imu_com_config(c: &ImuComConfigRaw) -> String {
    format!(
        "ImuComConfig {{ \
        \n\t\t\tmultiplexer_addr: {:?}, \
        \n\t\t\tmultiplexer_bus: {}, \
        \n\t\t\tsensor_addr: {} \
        }}",
        c.multiplexer_addr,
        c.multiplexer_bus,
        c.sensor_addr,
    )
}

#[derive(Deserialize)]
struct ImuAccelConfigRaw {
    scale: u8,
}

fn format_imu_accel_config(c: &ImuAccelConfigRaw) -> String {
    format!(
        "ImuAccelConfig {{ \
        \n\t\t\tscale: {} \
        }}",
        c.scale,
    )
}

#[derive(Deserialize)]
struct ImuGyroConfigRaw {
    scale: u8,
}

fn format_imu_gyro_config(c: &ImuGyroConfigRaw) -> String {
    format!(
        "ImuGyroConfig {{ \
        \n\t\t\tscale: {} \
        }}",
        c.scale,
    )
}

#[derive(Deserialize)]
struct ImuPoseConfigRaw {
    s2f: [[f32;3];3],
    origin_f: [f32;3],
}

fn format_imu_pose_config(c: &ImuPoseConfigRaw) -> String {
    format!(
        "ImuPoseConfig {{ \
        \n\t\t\ts2f: {}, \
        \n\t\t\torigin_f: {} \
        }}",
        format_matrix3(c.s2f),
        format_vector3(c.origin_f),
    )
}

fn format_f32(f: f32) -> String{
    if f.fract() == 0.0 {
        format!("{:.1}", f)   // whole number: force "1.0" not "1"
    } else {
        format!("{}", f)      // has a fractional part: default formatting preserves precision
    }
}

fn format_vector3(v: [f32;3]) -> String {
    format!( "nalgebra::Vector3::<f32>::new( \
    \n\t\t\t\t{},{},{})",
        format_f32(v[0]),
        format_f32(v[1]),
        format_f32(v[2]),
    )
}

fn format_matrix3(m: [[f32;3];3]) -> String {
    format!( "nalgebra::Matrix3::<f32>::new( \
    \n\t\t\t\t{},{},{}, \
    \n\t\t\t\t{},{},{}, \
    \n\t\t\t\t{},{},{})",
        format_f32(m[0][0]),format_f32(m[0][1]),format_f32(m[0][2]),
        format_f32(m[1][0]),format_f32(m[1][1]),format_f32(m[1][2]),
        format_f32(m[2][0]),format_f32(m[2][1]),format_f32(m[2][2]),
    )
}

#[derive(Deserialize)]
struct ImuConfigRaw {
    communication:  ImuComConfigRaw,
    accelerometer:  ImuAccelConfigRaw,
    gyroscope:      ImuGyroConfigRaw,
    pose:           ImuPoseConfigRaw,
}

fn format_imu_config(c: &ImuConfigRaw) -> String {
    format!(
        "\n\tImuConfig {{ \
        \n\t\tcommunication: {}, \
        \n\t\taccelerometer: {}, \
        \n\t\tgyroscope: {}, \
        \n\t\tpose: {} \
        }}",
        format_imu_com_config(&c.communication),
        format_imu_accel_config(&c.accelerometer),
        format_imu_gyro_config(&c.gyroscope),
        format_imu_pose_config(&c.pose),
    )
}

fn main() {
    let config_dir = "../sensor-config";
    println!("cargo:rerun-if-changed={config_dir}");

    let manifest_path = format!("{config_dir}/manifest.toml");
    let manifest_str = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("failed to read {manifest_path}: {e}"));
    let manifest: Manifest = toml::from_str(&manifest_str)
        .unwrap_or_else(|e| panic!("failed to parse {manifest_path}: {e}"));

    let mut generated = String::new();
    
    // get number of IMUs from number specified in manifest
    generated.push_str(&format!("pub const N_IMUS: usize = {};\n", manifest.sensors.len()));

    // start definition of imu configuration array
    generated.push_str("pub const IMU_CONFIGS: [ImuConfig; N_IMUS] = [");

    for filename in &manifest.sensors {
        print!("loading {filename}");
        let path = format!("{config_dir}/{filename}.toml");
        let contents = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("failed to read {path}: {e}"));
        let cfg: ImuConfigRaw = toml::from_str(&contents)
            .unwrap_or_else(|e| panic!("failed to parse {path}: {e}"));
        generated.push_str("    ");
        generated.push_str(&format_imu_config(&cfg));
        generated.push_str(",\n");
    }

    generated.push_str("];\n");

    let out_dir = env::var("OUT_DIR").unwrap();
    fs::write(Path::new(&out_dir).join("sensor_configs.rs"), generated).unwrap();
}