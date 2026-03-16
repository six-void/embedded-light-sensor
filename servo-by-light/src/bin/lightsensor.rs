//! # VEML7700 Light Sensor Example
//!
//! Reads lux from a VEML7700 over I2C (SDA=GP0, SCL=GP1).
//! Logs lux value over USB serial (ACM0).
//! LED on if lux < 39, off otherwise.

#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;

use rp235x_hal::fugit::RateExtU32;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use core::fmt::Write;
use heapless::String;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// VEML7700 I2C address
const VEML7700_ADDR: u8 = 0x10;

// VEML7700 registers
const REG_ALS_CONF: u8 = 0x00;
const REG_ALS_WH: u8 = 0x01;
const REG_ALS_WL: u8 = 0x02;
const REG_POW_SAV: u8 = 0x03;
const REG_ALS: u8 = 0x04;

// Config bytes for IT=100ms, gain=1/8 → confValues[100][1/8] = [0x00, 0x10]
const VEML_CONF: [u8; 2] = [0x00, 0x10];

// Gain multiplier for IT=100ms, gain=1/8 → gainValues[100][1/8] = 0.4608
// Using fixed-point: 0.4608 * 10000 = 4608, divide by 10000 at end
const GAIN_SCALED: u32 = 4608;
const GAIN_DIVISOR: u32 = 10000;

fn veml7700_init<I: I2c>(i2c: &mut I) {
    // ALS config: IT=100ms, gain=1/8
    i2c.write(VEML7700_ADDR, &[REG_ALS_CONF, VEML_CONF[0], VEML_CONF[1]]).ok();
    // Clear high/low threshold windows
    i2c.write(VEML7700_ADDR, &[REG_ALS_WH, 0x00, 0x00]).ok();
    i2c.write(VEML7700_ADDR, &[REG_ALS_WL, 0x00, 0x00]).ok();
    // Power save off
    i2c.write(VEML7700_ADDR, &[REG_POW_SAV, 0x00, 0x00]).ok();
}

fn veml7700_read_lux<I: I2c>(
    i2c: &mut I,
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
) -> u32 {
    // Wait 40ms for sensor integration settling
    timer.delay_ms(40);

    // Read 2 bytes from ALS register
    let mut buf = [0u8; 2];
    i2c.write_read(VEML7700_ADDR, &[REG_ALS], &mut buf).ok();

    // Raw count: low byte + high byte * 256
    let raw = buf[0] as u32 + buf[1] as u32 * 256;

    // Apply gain: lux = raw * 0.4608, rounded to nearest integer
    (raw * GAIN_SCALED + GAIN_DIVISOR / 2) / GAIN_DIVISOR
}

fn serial_write_all(serial: &mut SerialPort<hal::usb::UsbBus>, s: &str) {
    let mut bytes = s.as_bytes();
    while !bytes.is_empty() {
        match serial.write(bytes) {
            Ok(n) => bytes = &bytes[n..],
            Err(_) => break,
        }
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // --- USB Serial setup ---
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("rp235x")
            .product("Light Sensor")
            .serial_number("1")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    // --- I2C setup: SDA=GP0, SCL=GP1, PullUp, 400kHz ---
    let sda: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio0.reconfigure();
    let scl: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio1.reconfigure();

    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda,
        scl,
        400_000u32.Hz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // --- LED setup ---
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // --- Init VEML7700 sensor ---
    veml7700_init(&mut i2c);

    //TODO: Remove or make this non-blocking
    // --- Wait for USB host to connect ---
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            let _ = serial.read(&mut buf);
        }
        if usb_dev.state() == UsbDeviceState::Configured {
            break;
        }
    }

    serial_write_all(&mut serial, "VEML7700 Light Sensor ready\r\n");

    let mut state: CurtainState = CurtainState::Opened; 

    // --- Main loop ---
    loop {
        // Poll USB across the ~1 second wait (100 x 10ms = 1000ms)
        // This keeps USB alive — long delay_ms calls without polling will disconnect
        for _ in 0..100 {
            if usb_dev.poll(&mut [&mut serial]) {
                let mut buf = [0u8; 64];
                let _ = serial.read(&mut buf);
            }
            timer.delay_ms(10);
        }

        let lux = veml7700_read_lux(&mut i2c, &mut timer);

        // Log over USB serial
        let mut s: String<64> = String::new();
        write!(s, "Lux: {}\r\n", lux).ok();
        serial_write_all(&mut serial, s.as_str());

        // LED on if dark (< 39 lux), off otherwise
        match state {
            CurtainState::Opened=>{
                if lux < 9 {
                    //TODO: open and stop
                    led_pin.set_high().unwrap();
                    state = CurtainState::Opened;
                } 
            }
            CurtainState::Closed => {
                if lux > 40 {
                    //TODO: close and stop
                    led_pin.set_low().unwrap();
                    state = CurtainState::Closed;
                } 
            },
        }
    }
}

enum CurtainState {
    Opened,Closed
}

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"VEML7700 Light Sensor"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
