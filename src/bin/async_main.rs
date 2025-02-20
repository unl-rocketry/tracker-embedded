#![no_main]
#![no_std]

use core::cell::RefCell;
use alloc::string::String;

use embedded_hal_bus::i2c::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, i2c::{self, master::I2c}};
use log::info;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use pololu_tic::{base::TicBase, TicI2C, TicProduct, TicStepMode};

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let sda = peripherals.GPIO18;
    let scl = peripherals.GPIO19;

    let _ = spawner;

    let i2c_bus = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default()
            .with_timeout(i2c::master::BusTimeout::Maximum)
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl)
    .into_async();

    let i2c_bus = RefCell::new(i2c_bus);

    let mut motor_horizontal = pololu_tic::TicI2C::new_with_address(
        RefCellDevice::new(&i2c_bus),
        TicProduct::Tic36v4,
        14,
    );

    let mut motor_vertical = pololu_tic::TicI2C::new_with_address(
        RefCellDevice::new(&i2c_bus),
        TicProduct::Tic36v4,
        15,
    );

    Timer::after(Duration::from_secs(1)).await;

    setup_motor(&mut motor_horizontal);

    motor_horizontal.set_target_velocity(7000000);

    loop {
        Timer::after(Duration::from_millis(100)).await;
        motor_horizontal.reset_command_timeout();
        info!("{}", motor_horizontal.current_position());
    }
}

fn setup_motor<I: embedded_hal::i2c::I2c>(motor: &mut TicI2C<I>) {
    motor.set_current_limit(2000);
    motor.halt_and_set_position(0);
    motor.set_max_decel(7000000);
    motor.set_max_accel(7000000);
    motor.set_max_speed(7000000);
    motor.set_starting_speed(7000000);
    motor.set_step_mode(TicStepMode::Full);
    motor.exit_safe_start();
}
