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
    panic
};
use esp_println::println;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::rmt::Rmt;
// use esp_hal::delay::Delay;
// use esp_hal::peripherals::Peripherals;
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smart_leds::{RGB8, SmartLedsWrite as _};
use {esp_backtrace as _, esp_println as _};

use esp_hal::i2c::master::{I2c, Config};

use icm20948::Icm20948;
use imu_traits::Imu;

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

    let mut timestamp =0u64;
    let loop_duration_us = 200; 

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);
    let rmt = Rmt::new(_peripherals.RMT, Rate::from_mhz(80)).unwrap();

    let i2c_config = Config::default();
    let i2c = I2c::new(_peripherals.I2C0, i2c_config)
        .expect("Failed to initialize i2c")
        .with_sda(_peripherals.GPIO2)
        .with_scl(_peripherals.GPIO3);
    let mut imu = Icm20948::new(i2c,0x69);
    match imu.init() {
        Ok(_) => {
            info!("Init succeeded!");
        }
        Err(e) => {
            info!("Init failed: {}", defmt::Debug2Format(&e));
            panic!("Manual panic after init failure");
        }
    }
    let (mut phi, mut theta, mut psi): (f32, f32, f32) = (0., 0., 0.);
    
    // let system = peripherals.SYSTEM.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);

    // let imu_delay = 10;
    // let interface = I2cInterface::default(i2c);
    // let mut imu = Icm20948Driver::new(interface).unwrap();
    // imu.init(&mut imu_delay).unwrap();

    let mut led_buf = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, _peripherals.GPIO8, &mut led_buf);
    const LEVEL: u8 = 10;
    let mut color = RGB8::default();
    color.r = LEVEL;

    info!("reading imu data");
    loop {
        led.write([color].into_iter()).unwrap();
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_micros(loop_duration_us) {}

        let (ax, ay, az) = imu.read_accelerometer_g()
            .expect("failed to read accelerometer");
        
        let (gx, gy, gz) = imu.read_gyroscope_dps()
            .expect("failed to read gyroscope");

        (phi, theta, psi) = imu.attitude_from_direct_integration((phi, theta, psi), loop_duration_us)
            .expect("failed to calculate attitude by integration");

        // println!("{},{},{},{},{},{},{}", 
        //     timestamp, ax, ay, az, gx, gy, gz);
        println!("{},{},{},{},{},{},{},{},{},{}", 
                    timestamp, ax, ay, az, gx, gy, gz, phi, theta, psi);
        
        let tmp = color.r;
        color.r = color.b;
        color.b = color.g;
        color.g = tmp;

        timestamp += loop_duration_us;
    }
}