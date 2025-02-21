use crate::{calibrate_vertical, STEPS_PER_DEGREE_HORIZONTAL, STEPS_PER_DEGREE_VERTICAL};
use alloc::string::String;
use esp_println::println;
use mma8x5x::ic::Mma8451;
use mma8x5x::{mode, Mma8x5x};
use pololu_tic::{TicBase, TicI2C};

#[derive(Debug)]
pub enum ParseErr {
    Empty,
}

pub async fn parse_command<I: embedded_hal::i2c::I2c>(
    motor_vertical: &mut TicI2C<I>,
    motor_horizontal: &mut TicI2C<I>,
    accel: &mut Mma8x5x<I, Mma8451, mode::Active>,
    input: String,
) -> Result<(), ParseErr> {
    if input.len() == 1 {
        println!("ERR");
        return Err(ParseErr::Empty);
    }

    let arg1: &str = "stuff";

    match input.as_str() {
        "DVER" => {
            todo!()
        }
        "DHOR" => {
            todo!()
        }
        "CALV" => match arg1 {
            "SET" => {
                motor_vertical
                    .halt_and_set_position(0)
                    .expect("TODO: panic message");
            }
            _ => calibrate_vertical(motor_vertical, accel).await,
        },
        "CALH" => {
            motor_horizontal
                .halt_and_set_position(0)
                .expect("TODO: panic message");
        }
        "MOVV" => {
            todo!()
        }
        "MOVH" => {
            todo!()
        }
        "GETP" => {
            let vertical_position: f32 = motor_vertical.current_position().unwrap() as f32
                / STEPS_PER_DEGREE_VERTICAL as f32;
            let mut horzontal_position: f32 = motor_horizontal.current_position().unwrap() as f32
                / STEPS_PER_DEGREE_HORIZONTAL as f32;
            while horzontal_position > 180.0 {
                horzontal_position -= 360.0;
            }
            while horzontal_position < 180.0 {
                horzontal_position += 360.0;
            }
            println!("{} {}", vertical_position, horzontal_position);
        }
        "INFO" => {
            let command_list = [
                "DVER INT",
                "DHOR INT",
                "CALV {SET}",
                "CALH",
                "MOVV INT",
                "MOVH INT",
                "GETP",
                "SSPD INT {VER INT} {HOR INT} {RST}",
                "GSPD",
                "HALT",
            ];
            for command in command_list {
                println!("  {}", command);
            }
        }
        "SSPD" => {
            todo!()
        }
        "GSPD" => {
            println!(
                "{} {}",
                motor_vertical.max_speed().unwrap(),
                motor_horizontal.max_speed().unwrap()
            );
        }
        "HALT" => {
            motor_vertical.halt_and_hold().unwrap();
            motor_horizontal.halt_and_hold().unwrap();
        }
        _ => {}
    }

    Ok(())
}
