use crate::{
    calibrate_vertical, get_delta_angle, get_relative_angle, STEPS_PER_DEGREE_HORIZONTAL,
    STEPS_PER_DEGREE_VERTICAL,
};
use alloc::string::String;
use esp_println::println;
use mma8x5x::ic::Mma8451;
use mma8x5x::{mode, Mma8x5x};
use pololu_tic::{TicBase, TicI2C};

#[derive(Debug, thiserror::Error)]
pub enum ParseErr {
    #[error("the command was empty")]
    Empty,
    #[error("the number could not be parsed")]
    InvalidNumber,
    #[error("not enough arguments were provided")]
    TooFewArguments,
    #[error("the motor controllers experienced an error")]
    InternalError(#[from] pololu_tic::TicHandlerError),
}

pub async fn parse_command<I: embedded_hal::i2c::I2c>(
    motor_vertical: &mut TicI2C<I>,
    motor_horizontal: &mut TicI2C<I>,
    accel: &mut Mma8x5x<I, Mma8451, mode::Active>,
    mut input: String,
) -> Result<(), ParseErr> {
    if input.len() == 1 {
        println!("ERR");
        return Err(ParseErr::Empty);
    }

    input = input.to_ascii_uppercase();
    let mut arguments = input.split_whitespace();

    match arguments.next().unwrap() {
        "DVER" => {
            let target_pos = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) if n < 90.0 || n > 0.0 => n,
                _ => return Err(ParseErr::InvalidNumber),
            };

            motor_vertical
                .set_target_position((target_pos * STEPS_PER_DEGREE_VERTICAL as f32) as i32)?;
        }
        "DHOR" => {
            let target_pos = match arguments.next().unwrap().parse::<f32>() {
                Ok(n) if n < 180.0 || n > -180.0 => n,
                _ => return Err(ParseErr::InvalidNumber),
            };
            let angle_steps = get_delta_angle(get_relative_angle(motor_horizontal), target_pos)
                * STEPS_PER_DEGREE_HORIZONTAL as f32;
            let move_to = motor_horizontal.current_position()? as f32 + angle_steps;
            motor_horizontal.set_target_position(move_to as i32)?;
        }
        "CALV" => match arguments.next() {
            Some("SET") => {
                motor_vertical.halt_and_set_position(0)?;
            }
            _ => calibrate_vertical(motor_vertical, accel).await,
        },
        "CALH" => {
            motor_horizontal.halt_and_set_position(0)?;
        }
        "MOVV" => {
            todo!()
        }
        "MOVH" => {
            todo!()
        }
        "GETP" => {
            let vertical_position: f32 =
                motor_vertical.current_position()? as f32 / STEPS_PER_DEGREE_VERTICAL as f32;
            let mut horizontal_position: f32 =
                motor_horizontal.current_position()? as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32;
            while horizontal_position > 180.0 {
                horizontal_position -= 360.0;
            }
            while horizontal_position < -180.0 {
                horizontal_position += 360.0;
            }
            println!("{} {}", vertical_position, horizontal_position);
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
