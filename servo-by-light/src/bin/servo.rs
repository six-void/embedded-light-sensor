#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::pwm::SetDutyCycle;
use hal::fugit::RateExtU32;
use panic_halt as _;
use rp235x_hal::{self as hal, clocks::init_clocks_and_plls, pac, pwm, watchdog::Watchdog, Sio};

// Required by rp235x-hal
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000;

/// Convert a desired pulse width in microseconds to a PWM compare value.
/// PWM runs at 50Hz (20ms period), top = 20_000, each count = 1µs.
fn us_to_pwm(us: u16) -> u16 {
    us
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up PWM slice 0, channel A → GP0
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm1;

    // 125MHz clock / 125 divider = 1MHz → 1µs per tick
    pwm.set_ph_correct();
    pwm.set_div_int(125);
    pwm.set_div_frac(0);
    // 50Hz: 1_000_000 / 50 = 20_000 ticks per period
    pwm.set_top(20_000);
    pwm.enable();

    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio2);

    // --- Sweep the servo: left → center → right → center → repeat ---
    let positions: [u16; 4] = [1000, 1500, 2000, 1500]; // µs

    loop {
        for &pulse_us in positions.iter() {
            channel.set_duty_cycle(us_to_pwm(pulse_us)).unwrap();

            // Hold each position for ~1 second (busy loop)
            // Replace with a proper timer/delay in real projects
            cortex_m::asm::delay(125_000_000);
        }
    }
}
