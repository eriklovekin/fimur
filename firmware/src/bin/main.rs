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

use esp_hal::i2c::master::{
    I2c, 
    Config,
};

use icm20948::Icm20948;
use imu_traits::{
    ImuWithAdustableScale
};
use fimur::filter::{
    Filter,
};

use xca9548a::{
    Xca9548a, 
    SlaveAddr
};

use fusion_core::config::{
    N_IMUS,
    IMU_CONFIGS,
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

    let mut timestamp: u64 = 0;

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);
    let rmt = Rmt::new(
        _peripherals.RMT, Rate::from_mhz(80)).unwrap();

    let i2c_config = Config::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(_peripherals.I2C0, i2c_config)
        .expect("Failed to initialize i2c")
        .with_sda(_peripherals.GPIO2)
        .with_scl(_peripherals.GPIO3);
    // RefCell needed so multiple multiplexers or I2C devices can share the same bus
    let i2c_bus = RefCell::new(i2c);

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
    let ch5_bus = RefCell::new(parts.i2c5);
    let ch6_bus = RefCell::new(parts.i2c6);
    let ch7_bus = RefCell::new(parts.i2c7);

    let sensors: [Icm20948<_>; N_IMUS] = core::array::from_fn(|i| {
        let cfg = &IMU_CONFIGS[i];
        let b_cfg = cfg.communication.multiplexer_bus;
        let bus = match b_cfg {
        0 => &ch0_bus,
        1 => &ch1_bus,
        2 => &ch2_bus,
        3 => &ch3_bus,
        4 => &ch4_bus,
        5 => &ch5_bus,
        6 => &ch6_bus,
        7 => &ch7_bus,
        _ => panic!("unsupported mux channel: {b_cfg}")
        };
        let mut s_cfg = Icm20948::new_with_mount(
            RefCellDevice::new(bus),
            cfg.communication.sensor_addr, // must be (false,false,false)
            cfg.pose.origin_f,
            cfg.pose.s2f,
        );
        s_cfg.set_accelerometer_scale(
            cfg.accelerometer.scale
        ).unwrap_or_else(|e| panic!(
        "failed to set accelerometer range for sensor {}: {:?}",
        i, e
        ));
    s_cfg.set_gyroscope_scale(
            cfg.gyroscope.scale
        ).unwrap_or_else(|e| panic!(
        "failed to set gyroscope range for sensor {}: {:?}",
        i, e
        ));
        s_cfg
    });

    let mut f = Filter::new(
        sensors
    );

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

        // Read all sensors
        f.read_all();

        // Estimate virtual measurements
        f.colocated_coaligned_avg();

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