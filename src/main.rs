#![warn(unused_extern_crates)]
#![feature(isqrt)]
#![no_std]
#![no_main]

mod timesource;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embedded_sdmmc::{Mode, SdCard};
use heapless::String;
use timesource::DummyTimesource;
use ufmt::derive::uDebug;
use ufmt::{uDebug, uDisplay};

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_sync::channel;
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        NoopMutex,
    },
    channel::Channel,
    mutex,
};
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{AnyInput, AnyOutput, GpioPin, Io, Level, Output, Pull, NO_PIN},
    i2c::I2C,
    peripherals::{Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{self, master::Spi, FullDuplexMode},
    system::SystemControl,
    time,
    timer::timg::TimerGroup,
    Async, Blocking,
};
use esp_println::println;
use mpu6050_dmp::sensor_async::Mpu6050;
use static_cell::StaticCell;

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
        let accel = match Mpu6050::accel(&mut imu).await {
            Ok(accel) => accel,
            Err(e) => {
                log::error!("Error reading acceleration");
                continue;
            }
        };
        let x = accel.x() as i32;
        let y = accel.y() as i32;
        let z = accel.z() as i32;
        log_channel
            .send(LogMessage {
                acceleration: Vector3 {
                    x: accel.x(),
                    y: accel.y(),
                    z: accel.z(),
                },
                time: Instant::now().as_ticks(),
            })
            .await;

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

static SPI_BUS: StaticCell<NoopMutex<RefCell<spi::master::Spi<'static, SPI2, FullDuplexMode>>>> =
    StaticCell::new();

#[embassy_executor::task]
async fn sd_card(
    sdcard: SdCard<
        SpiDevice<
            'static,
            NoopRawMutex,
            Spi<'static, SPI2, FullDuplexMode>,
            Output<'static, GpioPin<19>>,
        >,
        Delay,
    >,
) {
    // Get the card size (this also triggers card initialisation because it's not been done yet)
    let num_bytes = loop {
        match sdcard.num_bytes() {
            Ok(num) => break num,
            Err(_) => {
                log::error!("Failed to connect to sdcard, trying again in 1s");
                Timer::after_secs(1).await;
            }
        }
    };
    println!("Card size is {} bytes", num_bytes);
    // Now let's look for volumes (also known as partitions) on our block device.
    // To do this we need a Volume Manager. It will take ownership of the block device.
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource);
    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    println!("Volume 0: {:?}", volume0);
    // Open the root directory (mutably borrows from the volume).
    let mut root_dir = volume0.open_root_dir().unwrap();

    {
        // Open a file called "MY_FILE.TXT" in the root directory
        // This mutably borrows the directory.
        let mut my_file = root_dir
            .open_file_in_dir("config.txt", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        // Print the contents of the file, assuming it's in ISO-8859-1 encoding
        while !my_file.is_eof() {
            let mut buffer = [0u8; 32];
            let num_read = my_file.read(&mut buffer).unwrap();
            let str = core::str::from_utf8(&buffer[0..num_read]).unwrap();
            for line in str.lines() {
                println!("{str}");
            }
            // for b in &buffer[0..num_read] {
            //     println!("{}", *b as char);
            // }
        }
    }

    let file_count: RefCell<usize> = RefCell::new(0);
    root_dir
        .iterate_dir(|_| *file_count.borrow_mut() += 1)
        .unwrap();
    let mut s: String<11> = heapless::String::new();
    ufmt::uwrite!(s, "LOG_{}.txt", *file_count.borrow()).unwrap();

    let mut file = root_dir
        .open_file_in_dir(s.as_str(), Mode::ReadWriteCreateOrTruncate)
        .unwrap();

    loop {
        let data = log_channel.receive().await;
        let mut s: String<128> = heapless::String::new();
        ufmt::uwrite!(s, "{}\n", data).unwrap();
        println!("writing: {s}");
        file.write(s.as_bytes()).unwrap();
        file.flush();
    }
}

#[derive(Debug, Clone, Default, uDebug)]
struct Vector3 {
    x: i16,
    y: i16,
    z: i16,
}

#[derive(uDebug)]
struct LogMessage {
    time: u64,
    acceleration: Vector3,
}

impl uDisplay for LogMessage {
    fn fmt<W>(&self, fmt: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(fmt, "{},{},{},{}", self.time, self.acceleration.x, self.acceleration.y, self.acceleration.z)
    }
}

static log_channel: Channel<CriticalSectionRawMutex, LogMessage, 64> = Channel::new();

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

    // embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(SPI_BUS, cs)
    let i2c = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio5,
        io.pins.gpio4,
        400_u32.kHz(),
        &clocks,
    );
    spawner.spawn(imu(i2c)).ok();

    spawner
        .spawn(blink_led(led, AnyInput::new(io.pins.gpio0, Pull::Up)))
        .ok();

    let sck = Some(io.pins.gpio23); // 34
    let mosi = Some(io.pins.gpio21); // 32
    let miso = Some(io.pins.gpio22); // 35

    let cs = Output::new(io.pins.gpio19, Level::High); // 32

    let spi = esp_hal::spi::master::Spi::new(
        peripherals.SPI2,
        400u32.kHz(),
        esp_hal::spi::SpiMode::Mode0,
        &clocks,
    )
    .with_pins(sck, mosi, miso, NO_PIN);

    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(spi_bus, cs);
    let sdcard = embedded_sdmmc::SdCard::new(spi, Delay);

    spawner.spawn(sd_card(sdcard)).ok();
}
