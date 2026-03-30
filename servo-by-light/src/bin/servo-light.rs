//! # VEML7700 Light Sensor Example
//!
//! Reads lux from a VEML7700 over I2C (SDA=GP0, SCL=GP1).
//! Logs lux value over USB serial (ACM0).
//! Closes curtain (spin left 5s) if lux < 9, opens (spin right 5s) if lux > 19.
//! 10 lux hysteresis buffer prevents oscillation.

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

use embedded_hal::pwm::SetDutyCycle;
use rp235x_hal::{clocks::init_clocks_and_plls, pac, pwm, watchdog::Watchdog, Sio};

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

// Config bytes for IT=100ms, gain=1/8
const VEML_CONF: [u8; 2] = [0x00, 0x10];

// Gain multiplier fixed-point: 0.4608 * 10000 = 4608
const GAIN_SCALED: u32 = 4608;
const GAIN_DIVISOR: u32 = 10000;

// Lux thresholds with 10 lux hysteresis buffer
const LUX_CLOSE_THRESHOLD: u32 = 9;  // below this → close curtain
const LUX_OPEN_THRESHOLD: u32 = 19;  // above this (9 + 10) → open curtain

fn veml7700_init<I: I2c>(i2c: &mut I) {
    i2c.write(VEML7700_ADDR, &[REG_ALS_CONF, VEML_CONF[0], VEML_CONF[1]]).ok();
    i2c.write(VEML7700_ADDR, &[REG_ALS_WH, 0x00, 0x00]).ok();
    i2c.write(VEML7700_ADDR, &[REG_ALS_WL, 0x00, 0x00]).ok();
    i2c.write(VEML7700_ADDR, &[REG_POW_SAV, 0x00, 0x00]).ok();
}

fn veml7700_read_lux<I: I2c>(
    i2c: &mut I,
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
) -> u32 {
    timer.delay_ms(40);
    let mut buf = [0u8; 2];
    i2c.write_read(VEML7700_ADDR, &[REG_ALS], &mut buf).ok();
    let raw = buf[0] as u32 + buf[1] as u32 * 256;
    (raw * GAIN_SCALED + GAIN_DIVISOR / 2) / GAIN_DIVISOR
}

fn us_to_pwm(us: u16) -> u16 {
    us
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
    let mut peripherals = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(peripherals.TIMER0, &mut peripherals.RESETS, &clocks);

    let sio = hal::Sio::new(peripherals.SIO);
    let pins = hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // --- USB Serial setup ---
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        peripherals.USB,
        peripherals.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
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
    let sda_light: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio0.reconfigure();
    let scl_light: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio1.reconfigure();

    let mut i2c = hal::I2C::i2c0(
        peripherals.I2C0,
        sda_light,
        scl_light,
        400_000u32.Hz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );

    // --- LED setup ---
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // --- Init VEML7700 sensor ---
    veml7700_init(&mut i2c);

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

    // --- PWM setup: slice 1, channel A → GP2 ---
    let pwm_slices = pwm::Slices::new(peripherals.PWM, &mut peripherals.RESETS);
    let mut pwm = pwm_slices.pwm1;

    // 125MHz / 125 = 1MHz → 1µs per tick
    pwm.set_div_int(125);
    pwm.set_div_frac(0);
    // 50Hz: 1_000_000 / 50 = 20_000 ticks
    pwm.set_top(19_999);
    pwm.enable();

    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio2);

    let left   = 1000u16; // spin close direction
    let right  = 2000u16; // spin open direction
    let center = 1500u16; // stop

    serial_write_all(&mut serial, "VEML7700 Light Sensor ready\r\n");

    let mut state = CurtainState::Opened;

    // --- Main loop ---
    loop {
        // Poll USB for ~1 second between readings
        for _ in 0..100 {
            if usb_dev.poll(&mut [&mut serial]) {
                let mut buf = [0u8; 64];
                let _ = serial.read(&mut buf);
            }
            timer.delay_ms(10);
        }

        let lux = veml7700_read_lux(&mut i2c, &mut timer);

        let mut s: String<64> = String::new();
        write!(s, "Lux: {} State: {}\r\n", lux, state.as_str()).ok();
        serial_write_all(&mut serial, s.as_str());

        match state {
            CurtainState::Opened => {
                if lux < LUX_CLOSE_THRESHOLD {
                    serial_write_all(&mut serial, "Closing curtain...\r\n");
                    led_pin.set_high().unwrap();

                    // Spin close direction for 5 seconds
                    channel.set_duty_cycle(us_to_pwm(left)).unwrap();
                    for _ in 0..500 {
                        if usb_dev.poll(&mut [&mut serial]) {
                            let mut buf = [0u8; 64];
                            let _ = serial.read(&mut buf);
                        }
                        timer.delay_ms(10);
                    }

                    // Stop servo: brief neutral pulse then cut signal
                    channel.set_duty_cycle(us_to_pwm(center)).unwrap();
                    timer.delay_ms(200);
                    channel.set_duty_cycle(0).unwrap();

                    state = CurtainState::Closed;
                    serial_write_all(&mut serial, "Curtain closed.\r\n");

                    // Cooldown: 5 seconds before next lux reading
                    for _ in 0..500 {
                        if usb_dev.poll(&mut [&mut serial]) {
                            let mut buf = [0u8; 64];
                            let _ = serial.read(&mut buf);
                        }
                        timer.delay_ms(10);
                    }
                }
            }
            CurtainState::Closed => {
                if lux > LUX_OPEN_THRESHOLD {
                    serial_write_all(&mut serial, "Opening curtain...\r\n");
                    led_pin.set_low().unwrap();

                    // Spin open direction for 5 seconds
                    channel.set_duty_cycle(us_to_pwm(right)).unwrap();
                    for _ in 0..500 {
                        if usb_dev.poll(&mut [&mut serial]) {
                            let mut buf = [0u8; 64];
                            let _ = serial.read(&mut buf);
                        }
                        timer.delay_ms(10);
                    }

                    // Stop servo: brief neutral pulse then cut signal
                    channel.set_duty_cycle(us_to_pwm(center)).unwrap();
                    timer.delay_ms(200);
                    channel.set_duty_cycle(0).unwrap();

                    state = CurtainState::Opened;
                    serial_write_all(&mut serial, "Curtain opened.\r\n");

                    // Cooldown: 5 seconds before next lux reading
                    for _ in 0..500 {
                        if usb_dev.poll(&mut [&mut serial]) {
                            let mut buf = [0u8; 64];
                            let _ = serial.read(&mut buf);
                        }
                        timer.delay_ms(10);
                    }
                }
            }
        }
    }
}

#[derive(defmt::Format)]
enum CurtainState {
    Opened,
    Closed,
}

impl CurtainState {
    fn as_str(&self) -> &'static str {
        match self {
            CurtainState::Opened => "Opened",
            CurtainState::Closed => "Closed",
        }
    }
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
