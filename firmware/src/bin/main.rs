#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{
    info,
};
// use esp_hal::lp_core::LpCoreClockSource;
use esp_println::println;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::rmt::Rmt;
use esp_hal::time::{Duration, Instant, Rate};
use core::cell::RefCell;
use core::fmt::Write;
use heapless::String;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smart_leds::{RGB8, SmartLedsWrite as _};
use {esp_backtrace as _, esp_println as _};

use esp_hal::i2c::master::{I2c, Config};

use icm20948::Icm20948;
use imu_traits::{Imu, ImuWithAdustableScale};
use fimur::filter::{
    Filter,
};

use nalgebra::{
    Matrix3, 
    Vector3,
};

use xca9548a::{Xca9548a, SlaveAddr};
// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    info!("startup");

    let mut timestamp: u64 = 0;
    // let loop_duration_us: u32 = 200; 

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);
    let rmt = Rmt::new(
        _peripherals.RMT, Rate::from_mhz(80)).unwrap();

    let i2c_config = Config::default();
    let i2c = I2c::new(_peripherals.I2C0, i2c_config)
        .expect("Failed to initialize i2c")
        .with_sda(_peripherals.GPIO2)
        .with_scl(_peripherals.GPIO3);
    // RefCell needed so multiple multiplexers or I2C devices can share the same bus
    let i2c_bus = RefCell::new(i2c);

    let aligned: Matrix3<f32> = Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    // let cots_aligned: Matrix3<f32> = Matrix3::new(
    //      0.0, -1.0,  0.0,
    //     -1.0,  0.0,  0.0,
    //      0.0,  0.0, -1.0
    // );

    // let switch_address = SlaveAddr::Alternative(false,false,false);
    let switch_address = SlaveAddr::default();
    let i2c_switch = Xca9548a::new(
        RefCellDevice::new(&i2c_bus), switch_address);
    let parts = i2c_switch.split();
    
    // RefCell needed so multiple devices can share the same channel of the multiplexer
    let ch0_bus = RefCell::new(parts.i2c0);
    let ch1_bus = RefCell::new(parts.i2c1);
    let ch2_bus = RefCell::new(parts.i2c2);
    let ch3_bus = RefCell::new(parts.i2c3);
    let ch4_bus = RefCell::new(parts.i2c4);
    // let ch5_bus = RefCell::new(parts.i2c5);
    // let ch6_bus = RefCell::new(parts.i2c6);
    // let ch7_bus = RefCell::new(parts.i2c7);

    let imu1 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch0_bus),0x68,
        Vector3::<f32>::new(-0.01,0.0,0.0),
        aligned);
    let imu2 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch0_bus),0x69,
        Vector3::<f32>::new(0.01,0.0,0.0),
        aligned);

    let imu3 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch1_bus),0x68,
        Vector3::<f32>::new(-0.01,0.0,-0.0115),
        aligned);
    let imu4 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch1_bus),0x69,
        Vector3::<f32>::new(0.01,0.0,-0.0115),
        aligned);

    let imu5 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch2_bus),0x68,
        Vector3::<f32>::new(-0.01,0.0,-0.023),
        aligned);
    let imu6 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch2_bus),0x69,
        Vector3::<f32>::new(0.01,0.0,-0.023),
        aligned);

    let imu7 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch3_bus),0x68,
        Vector3::<f32>::new(-0.01,0.0,-0.0345),
        aligned);
    let imu8 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch3_bus),0x69,
        Vector3::<f32>::new(0.01,0.0,-0.0345),
        aligned);

    let imu9 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch4_bus),0x68,
        Vector3::<f32>::new(-0.01,0.0,-0.0460),
        aligned);
    let imu10 = Icm20948::new_with_mount(
        RefCellDevice::new(&ch4_bus),0x69,
        Vector3::<f32>::new(0.01,0.0,-0.0460),
        aligned);

    // let imu11 = Icm20948::new_with_mount(
    //     RefCellDevice::new(&ch5_bus),0x68,
    //     Vector3::<f32>::new(0.0,-0.03,0.0),
    //     cots_aligned);
    // let imu12 = Icm20948::new_with_mount(
    //     RefCellDevice::new(&ch5_bus),0x69,
    //     Vector3::<f32>::new(0.0,-0.03,-0.0115),
    //     cots_aligned);

    let mut f = Filter::new([
        imu1, imu2, imu3, imu4, imu5, imu6, 
        imu7, imu8, imu9, imu10, //imu11, imu12
        ]);
    f.init();
    
    for s in 0..f.get_n_sensors() {
        f.sensor(s).set_accelerometer_scale(0)
            .expect("failed to set accelerometer range");
        f.sensor(s).set_gyroscope_scale(0)
            .expect("failed to set gyroscope range");
        f.sensor(s).set_origin_f(Vector3::<f32>::new(0.0, 0.0, 0.01));
        f.sensor(s).set_rotation_dcm_s2f(Matrix3::<f32>::new(  
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0));
    }
    
    let mut led_buf = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(
        rmt.channel0, _peripherals.GPIO8, &mut led_buf);
    const LEVEL: u8 = 10;
    let mut color = RGB8::default();
    color.r = LEVEL;

    let mut output: String<8192> = String::new();

    info!("reading imu data");
    loop {
        led.write([color].into_iter()).unwrap();
        let loop_start = Instant::now();
        // while delay_start.elapsed() < Duration::from_micros(loop_duration_us as u64) {}

        // Read all sensors
        f.read_all();

        // Estimate virtual measurements
        f.colocated_coaligned_avg();
        // f.f_frame_avg();

        output.clear();
        write!(output,"{},",timestamp).ok();
        write!(output,"{}", f.  report_raw()).ok();
        write!(output,"{}", f.report_virtual_meas()).ok();
        println!("{}",output);
        let tmp = color.r;
        color.r = color.b;
        color.b = color.g;
        color.g = tmp;

        let loop_end = Instant::now();
        let elapsed: Duration = loop_end - loop_start;
        timestamp += elapsed.as_micros();
    }
}