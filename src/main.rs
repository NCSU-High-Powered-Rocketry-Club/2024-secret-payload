#![warn(unused_extern_crates)]
#![feature(isqrt)]

#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, gpio::{AnyInput, AnyOutput, Io, Level, Output, Pull, NO_PIN}, i2c::I2C, peripherals::{Peripherals, I2C0, SPI2}, prelude::*, spi::{self, FullDuplexMode}, system::SystemControl, timer::timg::TimerGroup, Async, Blocking
};
use esp_println::println;
use mpu6050_dmp::sensor_async::Mpu6050;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_200)).await;
    }
}

#[embassy_executor::task]
async fn blink_led(mut led: AnyOutput<'static>, mut button: AnyInput<'static>) {
    loop {
        button.wait_for_any_edge().await;
        led.toggle();
        Timer::after(Duration::from_millis(100)).await;
        // button.wait_for_rising_edge().await;
        // esp_println::println!("Hello from button");
        // led.toggle();
        // Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn imu(i2c: I2C<'static, I2C0, Async>) {
    println!("Starting");
    let address = 0b1101000;
    let mut imu = Mpu6050::new(i2c, address.into()).await.unwrap();
    println!("Starting 2");
    // imu.initialize_dmp(&mut Delay).await.unwrap();

    loop {
        let accel = Mpu6050::accel(&mut imu).await.unwrap();
        let x = accel.x() as i32;
        let y = accel.y() as i32;
        let z = accel.z() as i32;
        // println!("accel {x:>5} {y:>5} {z:>5}");
        let pow = x.pow(2) + y.pow(2) + z.pow(2);
        if pow <= 0 {
            // Integer overflow
            continue;
        }
        let len = pow.isqrt();
        // println!("len {len}");
        let len = len as f32;
        let x = x as f32 / len;
        let y = y as f32 / len;
        let z = z as f32 / len;
        println!("accel {x:>.5} {y:>.5} {z:>.5}");
        Timer::after(Duration::from_millis(10)).await;
    }

    // Mpu6050::initialize_dmp(&mut imu, &mut Delay).await;

    // imu.get_fifo_count().unwrap();
}

static SPI_BUS: StaticCell<NoopMutex<RefCell<spi::master::Spi<'static, SPI2, FullDuplexMode>>>> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = AnyOutput::new(io.pins.gpio2, Level::Low);

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    spawner.spawn(run()).ok();
    spawner.spawn(blink_led(led, AnyInput::new(io.pins.gpio0, Pull::Up))).ok();

    let i2c = I2C::new_async(peripherals.I2C0, io.pins.gpio5, io.pins.gpio4, 400_u32.kHz(), &clocks);

    // spawner.spawn(imu(i2c)).ok();

    let sck = Some(io.pins.gpio23); // 34
    let mosi = Some(io.pins.gpio21); // 32
    let miso = Some(io.pins.gpio22); // 35
    
    let cs = Output::new(io.pins.gpio19, Level::High); // 32

    let spi = esp_hal::spi::master::Spi::new(peripherals.SPI2, 400u32.kHz(), esp_hal::spi::SpiMode::Mode0, &clocks).with_pins(sck, mosi, miso, NO_PIN);

    // embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(SPI_BUS, cs)

    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(spi_bus, cs);
    let sdcard = embedded_sdmmc::SdCard::new(spi, Delay);
}