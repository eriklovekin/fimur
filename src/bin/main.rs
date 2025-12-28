#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::rmt::Rmt;
use esp_hal::time::{Duration, Instant, Rate};
// use esp_hal::{
//     gpio::lp_io::LowPowerOutputOpenDrain,
//     i2c::lp_i2c::LpI2c,
//     load_lp_code,
//     lp_core::{LpCore, LpCoreWakeupSource}};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smart_leds::{RGB8, SmartLedsWrite as _};
use {esp_backtrace as _, esp_println as _};

use esp_hal::i2c::master::{I2c, Config, Operation};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// const ADDRESS1: u32 = 0x5000_2000;
// const ADDRESS2: u32 = 0x5000_2004;
const ENCODER_ADDRESS: u8 = 0x06;
// const ENCODER_ADDRESS: u32 = 0b0000110;
// const ANG_H_REGISTER: u8 = 0x04;
const ANG_L_REGISTER: u8 = 0x03;

// fn read_encoder_angle(i2c: &mut LpI2c) -> Result<(f32), Error> {
//     let mut buffer = [0u8; 2];
//     // Send single-shot measurement command.
//     i2c.write_read(ENCODER_ADDRESS, &(ANG_L_REGISTER),&mut buffer);
//     let angle_raw = ((buffer[0] as u16) << 6) | (buffer[1] as u16 & 0xFC);
//     Ok((angle_raw))
// }

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let i2c_config = esp_hal::i2c::master::Config::default();
    let _peripherals = esp_hal::init(config);
    let rmt = Rmt::new(_peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let mut led_buf = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, _peripherals.GPIO8, &mut led_buf);
    const LEVEL: u8 = 10;
    let mut color = RGB8::default();
    color.r = LEVEL;

    let mut i2c = I2c::new(_peripherals.I2C0, i2c_config)
        .expect("Failed to initialize i2c")
        .with_sda(_peripherals.GPIO2)
        .with_scl(_peripherals.GPIO3);
    // `u8` is automatically converted to `I2cAddress::SevenBit`. The device
    // address does not contain the `R/W` bit!
    // let write_buffer = [0xAA];
    let mut read_buffer = [0u8; 22];
    // i2c.write(DEVICE_ADDR, &write_buffer)?;
    // i2c.read(DEVICE_ADDR, &mut read_buffer)?;
    // i2c.transaction(
    //     DEVICE_ADDR,
    //     &mut [
    //         Operation::Write(&write_buffer),
    //         Operation::Read(&mut read_buffer),
    //     ],
    // )?;
    // configure LP I2C
    // let i2c = LpI2c::new(
    //     _peripherals.LP_I2C0,
    //     LowPowerOutputOpenDrain::new(_peripherals.GPIO6),
    //     LowPowerOutputOpenDrain::new(_peripherals.GPIO7),
    //     Rate::from_khz(100),
    // );
    // let mut lp_core = LpCore::new(_peripherals.LP_CORE);
    // lp_core.stop();
    // info!("lp core stopped");
    // load code to LP core
    // let lp_core_code = load_lp_code!("./lp_main.rs");

    // start LP core
    // lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, i2c);
    // info!("lp core run");

    // let temp = ADDRESS1 as *mut f32;
    // let humid = ADDRESS2 as *mut f32;

    loop {
        // info!("Hello world!");
        led.write([color].into_iter()).unwrap();
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}

        i2c.write_read(ENCODER_ADDRESS, &[ANG_L_REGISTER], &mut read_buffer)
            .expect("failed to write_read i2c");

        let angle_raw: u16 = 
            ((((read_buffer[0] as u16) << 6)) |
            (((read_buffer[1] >> 2) & 0b0011_1111) as u16)) &
            0b0011_1111_1111_1111;
        
        let angle_deg: u32 = (angle_raw as u32)*360/16384;

        info!("High register: {} | Low register: {} | Raw Angle: {} | Deg Angle: {}", 
            read_buffer[0], 
            read_buffer[1], 
            angle_raw, 
            angle_deg
        );

        let tmp = color.r;
        color.r = color.b;
        color.b = color.g;
        color.g = tmp;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
