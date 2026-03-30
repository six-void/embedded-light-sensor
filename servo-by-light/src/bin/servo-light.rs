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
use embedded_hal::pwm::SetDutyCycle;

use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::pwm;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use core::fmt::Write;
use heapless::String;

// --- Boot / linker blocks ---

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"VEML7700 Light Sensor"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// --- Constants ---

const XTAL_FREQ_HZ: u32 = 12_000_000;

const VEML7700_ADDR: u8 = 0x10;
const REG_ALS_CONF: u8 = 0x00;
const REG_ALS_WH:   u8 = 0x01;
const REG_ALS_WL:   u8 = 0x02;
const REG_POW_SAV:  u8 = 0x03;
const REG_ALS:      u8 = 0x04;

/// IT=100ms, gain=1/8
const VEML_CONF: [u8; 2] = [0x00, 0x10];

/// Lux = raw * 0.4608; expressed as fixed-point * 10_000
const GAIN_SCALED:  u32 = 4_608;
const GAIN_DIVISOR: u32 = 10_000;

const LUX_CLOSE_THRESHOLD: u32 = 9;   // below → close curtain
const LUX_OPEN_THRESHOLD:  u32 = 19;  // above → open curtain (10 lux hysteresis)

/// PWM pulse widths in microseconds (125 MHz / 125 = 1 µs/tick, top = 19_999 → 50 Hz)
const PWM_LEFT:   u16 = 1_000; // spin-close direction
const PWM_CENTER: u16 = 1_500; // stop
const PWM_RIGHT:  u16 = 2_000; // spin-open direction

// --- Curtain state ---

#[derive(defmt::Format, Clone, Copy, PartialEq, Eq)]
enum CurtainState {
    Opened,
    Closed,
}

impl CurtainState {
    fn as_str(self) -> &'static str {
        match self {
            CurtainState::Opened => "Opened",
            CurtainState::Closed => "Closed",
        }
    }

    /// Pure transition: given current state + lux, return the next state.
    fn next(self, lux: u32) -> CurtainState {
        match self {
            CurtainState::Opened if lux < LUX_CLOSE_THRESHOLD => CurtainState::Closed,
            CurtainState::Closed if lux > LUX_OPEN_THRESHOLD  => CurtainState::Opened,
            _ => self,
        }
    }

    /// Which PWM pulse to drive the motor with for this transition.
    fn motor_pulse(self) -> u16 {
        match self {
            CurtainState::Closed => PWM_LEFT,   // closing
            CurtainState::Opened => PWM_RIGHT,  // opening
        }
    }

    fn log_message(self) -> &'static str {
        match self {
            CurtainState::Closed => "Closing curtain...\r\n",
            CurtainState::Opened => "Opening curtain...\r\n",
        }
    }

    fn done_message(self) -> &'static str {
        match self {
            CurtainState::Closed => "Curtain closed.\r\n",
            CurtainState::Opened => "Curtain opened.\r\n",
        }
    }
}

// --- VEML7700 driver ---

fn veml7700_init<I: I2c>(i2c: &mut I) {
    let _ = i2c.write(VEML7700_ADDR, &[REG_ALS_CONF, VEML_CONF[0], VEML_CONF[1]]);
    let _ = i2c.write(VEML7700_ADDR, &[REG_ALS_WH,   0x00, 0x00]);
    let _ = i2c.write(VEML7700_ADDR, &[REG_ALS_WL,   0x00, 0x00]);
    let _ = i2c.write(VEML7700_ADDR, &[REG_POW_SAV,  0x00, 0x00]);
}

fn veml7700_read_raw<I: I2c>(i2c: &mut I) -> u32 {
    let mut buf = [0u8; 2];
    let _ = i2c.write_read(VEML7700_ADDR, &[REG_ALS], &mut buf);
    buf[0] as u32 | ((buf[1] as u32) << 8)
}

fn raw_to_lux(raw: u32) -> u32 {
    (raw * GAIN_SCALED + GAIN_DIVISOR / 2) / GAIN_DIVISOR
}

fn veml7700_read_lux<I: I2c>(
    i2c: &mut I,
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
) -> u32 {
    timer.delay_ms(40);
    raw_to_lux(veml7700_read_raw(i2c))
}

// --- USB helpers ---

fn serial_write<B: UsbBus>(serial: &mut SerialPort<B>, s: &str) {
    let mut bytes = s.as_bytes();
    while !bytes.is_empty() {
        match serial.write(bytes) {
            Ok(n)  => bytes = &bytes[n..],
            Err(_) => break,
        }
    }
}

/// Poll USB, discarding any incoming bytes. Returns true if the device is configured.
fn usb_poll<B: UsbBus>(
    usb_dev: &mut UsbDevice<B>,
    serial: &mut SerialPort<B>,
) -> bool {
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        let _ = serial.read(&mut buf);
    }
    usb_dev.state() == UsbDeviceState::Configured
}

/// Delay `ms` milliseconds while keeping the USB stack alive.
fn delay_with_usb<B: UsbBus>(
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
    usb_dev: &mut UsbDevice<B>,
    serial: &mut SerialPort<B>,
    ms: u32,
) {
    let ticks = ms / 10;
    for _ in 0..ticks {
        usb_dev.poll(&mut [serial]);
        timer.delay_ms(10);
    }
}

// --- Motor helpers ---

fn motor_run<B: UsbBus>(
    channel: &mut impl SetDutyCycle,
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
    usb_dev: &mut UsbDevice<B>,
    serial: &mut SerialPort<B>,
    pulse: u16,
    duration_ms: u32,
) {
    channel.set_duty_cycle(pulse).unwrap();
    delay_with_usb(timer, usb_dev, serial, duration_ms);
    channel.set_duty_cycle(PWM_CENTER).unwrap();
    timer.delay_ms(200);
    channel.set_duty_cycle(0).unwrap();
}

// --- Entry point ---

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

    let mut timer = hal::Timer::new_timer0(
        peripherals.TIMER0,
        &mut peripherals.RESETS,
        &clocks,
    );

    let sio = hal::Sio::new(peripherals.SIO);
    let pins = hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // USB serial
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        peripherals.USB,
        peripherals.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
    ));
    let mut serial  = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("rp235x")
            .product("Light Sensor")
            .serial_number("1")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    // I2C on GP0/GP1 at 400 kHz
    let sda: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio0.reconfigure();
    let scl: hal::gpio::Pin<_, hal::gpio::FunctionI2C, hal::gpio::PullUp> =
        pins.gpio1.reconfigure();
    let mut i2c = hal::I2C::i2c0(
        peripherals.I2C0,
        sda,
        scl,
        400_000u32.Hz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );

    // LED on GP25
    let mut led = pins.gpio25.into_push_pull_output();

    // PWM slice 1, channel A → GP2; 125 MHz / 125 = 1 µs/tick, 50 Hz
    let mut pwm_slices = pwm::Slices::new(peripherals.PWM, &mut peripherals.RESETS);
    let mut pwm = pwm_slices.pwm1;
    pwm.set_div_int(125);
    pwm.set_div_frac(0);
    pwm.set_top(19_999);
    pwm.enable();
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio2);

    veml7700_init(&mut i2c);

    // Wait for USB host
    while !usb_poll(&mut usb_dev, &mut serial) {}

    serial_write(&mut serial, "VEML7700 Light Sensor ready\r\n");

    let mut state = CurtainState::Opened;

    loop {
        delay_with_usb(&mut timer, &mut usb_dev, &mut serial, 1_000);

        let lux  = veml7700_read_lux(&mut i2c, &mut timer);
        let next = state.next(lux);

        let mut s: String<64> = String::new();
        write!(s, "Lux: {} State: {}\r\n", lux, state.as_str()).ok();
        serial_write(&mut serial, s.as_str());

        if next != state {
            serial_write(&mut serial, next.log_message());
            match next {
                CurtainState::Closed => led.set_high().unwrap(),
                CurtainState::Opened => led.set_low().unwrap(),
            }

            motor_run(
                channel,
                &mut timer,
                &mut usb_dev,
                &mut serial,
                next.motor_pulse(),
                5_000,
            );

            serial_write(&mut serial, next.done_message());
            state = next;

            // Cooldown before next reading
            delay_with_usb(&mut timer, &mut usb_dev, &mut serial, 5_000);
        }
    }
}
