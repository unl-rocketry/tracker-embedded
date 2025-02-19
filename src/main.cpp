#include <Wire.h>
#include <Tic.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>
#include <cmath>

// ##### TOP LEVEL STUFF ##### //
// ########################### //


// #### PIN DEFINITIONS #### //
const int SDA_PIN = 18;
const int SCL_PIN = 19;

// #### MOTOR DRIVERS #### //
TicI2C motorHorizontal(14);
TicI2C motorVertical(15);

// Steps per degree for the motor drivers at the default stepping
const int TIC_STEPS_PER_DEGREE_VERTICAL     = 24;
const int TIC_STEPS_PER_DEGREE_HORIZONTAL   = 126;
const int TIC_SPEED_VERYSLOW                = 500000;   //only used on CALV so no need to add a second one
const int TIC_SPEED_DEFAULT_VERTICAL        = 7000000;
const int TIC_SPEED_DEFAULT_HORIZONTAL      = 7000000;
const int TIC_SPEED_MAX_VERTICAL            = 7000000;
const int TIC_SPEED_MAX_HORIZONTAL          = 7000000;

const int TIC_DECEL_DEFAULT = 2000000;

// #### ACCELEROMETER #### //
Adafruit_MMA8451 mma8451 = Adafruit_MMA8451();

// Offsets calculated manually from accelerometer data
const int ACC_OFFSET_X =  263;
const int ACC_OFFSET_Y = -194;
const int ACC_OFFSET_Z =  142;

// ##### END OF TOP LEVEL STUFF ##### //
// ################################## //

auto calculatePitch() -> double {
    mma8451.read();
    double x = double(mma8451.y - ACC_OFFSET_Y) / 4096;
    double y = double(mma8451.x - ACC_OFFSET_X) / 4096;
    double z = double(mma8451.z - ACC_OFFSET_Z) / 4096;

    // Calculate pitch from acceleration data
    return atan2(-x, pow(y, 2) + pow(z, 2) ) * 57.29577951;
}

void setupMotor(TicI2C motor, String whichMotor) {
    motor.setProduct(TicProduct::Tic36v4);
    motor.setCurrentLimit(2000);
    motor.setAgcFrequencyLimit(TicAgcFrequencyLimit::F675Hz);
    motor.haltAndSetPosition(0);
    Serial.println(motor.getMaxAccel());
    motor.setMaxDecel(TIC_DECEL_DEFAULT);
    motor.setMaxAccel(TIC_DECEL_DEFAULT);
    if (whichMotor == "Vert") {
        motor.setMaxSpeed(TIC_SPEED_DEFAULT_VERTICAL);
    } else if (whichMotor == "Horiz") {
        motor.setMaxSpeed(TIC_SPEED_DEFAULT_HORIZONTAL);
    } else
        motor.setMaxSpeed(min(TIC_SPEED_DEFAULT_VERTICAL, TIC_SPEED_DEFAULT_HORIZONTAL));
    motor.setStepMode(TicStepMode::Full);

    motor.exitSafeStart();
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);

    while (!Serial) delay(10);

    Wire.begin(SDA_PIN, SCL_PIN);

    delay(1000);

    Serial.println("Starting MMA8451");
    if (!mma8451.begin()) {
        Serial.println("Failed to start MMA8451!");
    } else {
        Serial.println("MMA8451 found!");
    }

    mma8451.setRange(MMA8451_RANGE_2_G);
    mma8451.setDataRate(mma8451_dataRate_t::MMA8451_DATARATE_50_HZ);

    // Set up motor driver(s)
    setupMotor(motorVertical, "Vert");
    setupMotor(motorHorizontal, "Horiz");
}

void calibrateVertical() {
    const float ZERO_CAL = 0.2;
    int targetVelocity;
    motorVertical.setMaxDecel(5000000);
    motorVertical.setMaxAccel(5000000);
    motorVertical.setStepMode(TicStepMode::Microstep8);

    // Find the zero position
    while (true) {
        double pitchSum = 0;
        for (int i = 0; i < 20; i++) {
            double pitch = calculatePitch();
            pitchSum += pitch;
            delay(20);
        }
        pitchSum /= 20;

        // Prevents movement from erroring out
        motorVertical.resetCommandTimeout();

        if (fabs(pitchSum - ZERO_CAL) < 3.0) {
            targetVelocity = -TIC_SPEED_VERYSLOW;
            delay(50);
        } else {
            targetVelocity = -TIC_SPEED_DEFAULT_VERTICAL;
        }

        if (pitchSum <= -0.02) {
            motorVertical.setTargetVelocity(targetVelocity);
        } else if (pitchSum >= 0.02) {
            motorVertical.setTargetVelocity(-targetVelocity);
        } else {
            motorVertical.haltAndSetPosition(0);
            break;
        }
        delay(200);
        motorVertical.setTargetVelocity(0);
    }
    motorVertical.setMaxDecel(TIC_DECEL_DEFAULT);
    motorVertical.setMaxAccel(TIC_DECEL_DEFAULT);
    motorVertical.setStepMode(TicStepMode::Full);
}

float getDeltaAngle(float curr_angle, float new_angle) {
    // Calculate most optimal difference in current and destination angle
    float diff = fmod((new_angle - curr_angle + 180), (float) 360) - 180; 
    if (diff < -180) { 
        return diff + 360; // if angle less than -180, switch direction 
    } else {
        return diff; 
    }
}

void parseCommand(String &input) {
    if (input.length() == 1) {
        Serial.println("ERR");
    }

    int indicies[32] = {};
    int indexIndicies = 0;

    int lastIndex = 0;

    while (true) {
        int nextIndex = input.indexOf(' ', lastIndex);
        if (nextIndex == -1) {
            break;
        }

        indicies[indexIndicies] = nextIndex;
        indexIndicies += 1;

        lastIndex = nextIndex + 1;
    }

    String command = input.substring(0, indicies[0]);
    command.toUpperCase();
    command.trim();
    Serial.print("Got command \"");
    Serial.print(command);
    Serial.println("\"");

    if (command == "DVER") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();
        if (indexIndicies == 1) {
            Serial.println("ERR");
            return;
        }

        float position = arg1.toFloat();
        if (fabs(position) > 90) {
            Serial.println("ERR");
            return;
        }

        motorVertical.setTargetPosition((int32_t) (position * float(TIC_STEPS_PER_DEGREE_VERTICAL)));

    } else if (command == "DHOR") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        if (indexIndicies == 1) {
            Serial.println("ERR");
            return;
        }

        float position = arg1.toFloat();
        if (fabs(position) > 180) {
            Serial.println("ERR");
            return;
        }

        // Get current position in steps
        float curr_pos = (float) motorHorizontal.getCurrentPosition() / (float) TIC_STEPS_PER_DEGREE_HORIZONTAL;
        
        while (curr_pos > 180) {
            curr_pos -= 360;
        }
        while (curr_pos < -180) {
            curr_pos += 360;
        }

        // Find the number of steps to get to destination
        int32_t angle_steps = (getDeltaAngle(curr_pos, position) * TIC_STEPS_PER_DEGREE_HORIZONTAL);
        
        motorHorizontal.setTargetPosition(motorHorizontal.getCurrentPosition() + angle_steps);

    } else if (command == "CALV") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        if (arg1 == "SET") {
            motorVertical.haltAndSetPosition(0);
        } else if (indicies[1] == 0) {
            calibrateVertical();
        } else {
            Serial.println("ERR");
            return;
        }

    } else if (command == "CALH") {
        motorHorizontal.haltAndSetPosition(0);

    } else if (command == "MOVV") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        if (indexIndicies == 1) {
            Serial.println("ERR");
            return;
        }

        auto steps_to_move = (int32_t) arg1.toInt();
        int32_t current_position = motorVertical.getCurrentPosition();
        int32_t move_to = current_position + steps_to_move;
        motorVertical.setTargetPosition(move_to);

    } else if (command == "MOVH") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        if (indexIndicies == 1) {
            Serial.println("ERR");
            return;
        }

        auto steps_to_move = (int32_t) arg1.toInt();
        int32_t current_position = motorHorizontal.getCurrentPosition();
        int32_t move_to = current_position + steps_to_move;
        motorHorizontal.setTargetPosition(move_to);

    } else if (command == "GETP") {
        float vertical_position = (float) motorVertical.getCurrentPosition() / (float) TIC_STEPS_PER_DEGREE_VERTICAL;
        float horizontal_position = (float) motorHorizontal.getCurrentPosition() / (float) TIC_STEPS_PER_DEGREE_HORIZONTAL;
        while (horizontal_position > 180) {
            horizontal_position -= 360;
        }
        while (horizontal_position < -180) {
            horizontal_position += 360;
        }
        Serial.printf("OK %g %g\n",vertical_position, horizontal_position);

    } else if (command == "INFO") {

        Serial.println("Command List:");
        String command_list[] = {"DVER INT", "DHOR INT", "CALV {SET}", "CALH", "MOVV INT", "MOVH INT", "GETP", "SSPD INT {VER INT} {HOR INT} {RST}", "GSPD"};
        for (String &command : command_list) {
            Serial.print("  ");
            Serial.println(command);
        }

    } else if (command == "SSPD") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        int64_t new_speed;

        if (indexIndicies == 3) {
            String arg2 = input.substring(indicies[1], indicies[2]);
            arg2.trim();
            new_speed = arg2.toInt();
        } else if (indexIndicies == 1 || ((arg1 == "VER" || arg1 == "HOR") && indexIndicies != 3)) {
            Serial.println("ERR");
            return;
        }

        if (arg1 == "VER") {
            if (new_speed > TIC_SPEED_MAX_VERTICAL) {
                new_speed = TIC_SPEED_MAX_VERTICAL;
            }
            motorVertical.setMaxSpeed(new_speed);
        } else if (arg1 == "HOR") {
            if (new_speed > TIC_SPEED_MAX_HORIZONTAL) {
                new_speed = TIC_SPEED_MAX_HORIZONTAL;
            }
            motorHorizontal.setMaxSpeed(new_speed);
        } else if (arg1 == "RST") {
            motorVertical.setMaxSpeed(TIC_SPEED_DEFAULT_VERTICAL);
            motorHorizontal.setMaxSpeed(TIC_SPEED_DEFAULT_HORIZONTAL);
        } else if (indicies[1] == 0) {
            motorVertical.setMaxSpeed(new_speed);
            motorHorizontal.setMaxSpeed(new_speed);
        } else {
            Serial.println("ERR");
            return;
        }

    } else if (command == "GSPD") {
        uint32_t vertical_speed;
        uint32_t Horizontal_speed;

        vertical_speed = motorVertical.getMaxSpeed();
        Horizontal_speed = motorHorizontal.getMaxSpeed();

        Serial.printf("OK %i %i\n",vertical_speed, Horizontal_speed);
        return;

    } else {
        Serial.println("ERR");
        return;
    }

    Serial.println("OK");
}

String commandString = "";
void loop() {
    // Prevents movement from erroring out
    motorHorizontal.resetCommandTimeout();
    motorVertical.resetCommandTimeout();

    if (Serial.available() > 0) {
        // Read in a string until a newline, not including the newline
        int byte = Serial.read();
        if (byte == '\n') {
            // The command string has been terminated
            commandString += ' ';
            parseCommand(commandString);
            commandString.clear();
        } else if (byte == '\b') {
            // Backspace, delete the last character in the string. Unless it's empty
            // then scream ig
            if (commandString.length() != 0) {
                commandString.remove(commandString.length() - 1);
                Serial.print(" \b");
            }
        } else if (byte != 0xFF) {
            // This is a regular valid character, add it to the string
            commandString += (char) byte;
        }
    }

    //Serial.printf("%i\n", motorVertical.getCurrentPosition());

    delay(10);
}
