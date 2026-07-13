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

    let mut timestamp: u32 = 0;
    let loop_duration_us: u32 = 200; 

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);
    let rmt = Rmt::new(_peripherals.RMT, Rate::from_mhz(80)).unwrap();

    let i2c_config = Config::default();
    let i2c = I2c::new(_peripherals.I2C0, i2c_config)
        .expect("Failed to initialize i2c")
        .with_sda(_peripherals.GPIO2)
        .with_scl(_peripherals.GPIO3);
    let i2c_bus = RefCell::new(i2c);
    let i2c_dev1 = RefCellDevice::new(&i2c_bus);
    let i2c_dev2 = RefCellDevice::new(&i2c_bus);
    let imu1 = Icm20948::new(i2c_dev1,0x68);
    let imu2 = Icm20948::new(i2c_dev2,0x69);

    let mut f = Filter::new(loop_duration_us,[imu1, imu2]);
    f.init();
    
    // IMU1 is on top of stack
    f.sensor(0).set_accelerometer_scale(3)
        .expect("failed to set accelerometer range");
    f.sensor(0).set_gyroscope_scale(3)
        .expect("failed to set gyroscope range");
    f.sensor(0).set_origin_f([0.0, 0.0, 0.0]);
    f.sensor(0).set_rotation_dcm_s2f([  [1.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0],
                                        [0.0, 0.0, 1.0]]);

    f.sensor(1).set_accelerometer_scale(3)
        .expect("failed to set accelerometer range");
    f.sensor(1).set_gyroscope_scale(3)
        .expect("failed to set gyroscope range");
    f.sensor(0).set_origin_f([0.0, 0.0, 0.01]);
    f.sensor(0).set_rotation_dcm_s2f([  [1.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0],
                                        [0.0, 0.0, 1.0]]);

    f.init_default_kinematics();   
    // f.init_block_process_noise_covariance();                     

    let mut led_buf = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, _peripherals.GPIO8, &mut led_buf);
    const LEVEL: u8 = 10;
    let mut color = RGB8::default();
    color.r = LEVEL;

    let mut output: String<8192> = String::new();

    info!("reading imu data");
    loop {
        led.write([color].into_iter()).unwrap();
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_micros(loop_duration_us as u64) {}

        // Read all sensors
        f.read_all();

        // Estimate virtual measurements
        // f.most_naive_filter_possible();

        // Estimate Position
        output.clear();
        write!(output,"{},",timestamp).ok();
        write!(output,"{}", f.report_raw()).ok();
        println!("{}",output);
        let tmp = color.r;
        color.r = color.b;
        color.b = color.g;
        color.g = tmp;

        timestamp += loop_duration_us;
    }
}