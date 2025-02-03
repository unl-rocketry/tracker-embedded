#include <Arduino.h>
#include <Wire.h>
#include <Tic.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>

// ##### TOP LEVEL STUFF ##### //
// ########################### //


// #### PIN DEFINITIONS #### //
const int SDA_PIN = 18;
const int SCL_PIN = 19;

// #### MOTOR DRIVERS #### //
TicI2C motorHorizontal(14);
TicI2C motorVertical(15);

// Steps per degree for the motor drivers at the default stepping
const int TIC_STEPS_PER_DEGREE  = 126;
const int TIC_SPEED_VERYSLOW    = 500000;
const int TIC_SPEED_DEFAULT     = 7000000;

// #### ACCELEROMETER #### //
Adafruit_MMA8451 mma8451 = Adafruit_MMA8451();

// Offsets calculated manually from accelerometer data
const int ACC_OFFSET_X =  263;
const int ACC_OFFSET_Y = -194;
const int ACC_OFFSET_Z =  142;

// ##### END OF TOP LEVEL STUFF ##### //
// ################################## //

double calculatePitch() {
    mma8451.read();
    double x = double(mma8451.y - ACC_OFFSET_Y) / 4096;
    double y = double(mma8451.x - ACC_OFFSET_X) / 4096;
    double z = double(mma8451.z - ACC_OFFSET_Z) / 4096;

    // Calculate pitch from acceleration data
    return atan2(-x, pow(y, 2) + pow(z, 2) ) * 57.29577951;
}

void setupMotor(TicI2C motor) {
    motor.setProduct(TicProduct::Tic36v4);
    motor.setCurrentLimit(1024);
    motor.setAgcFrequencyLimit(TicAgcFrequencyLimit::F675Hz);
    motor.haltAndSetPosition(0);
    // Default is 7,000,000
    motor.setMaxSpeed(7000000);
    motor.setStepMode(TicStepMode::Full);

    motor.exitSafeStart();
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);

    while (!Serial) delay(10);

    Wire.begin(SDA_PIN, SCL_PIN);

    delay(100);

    Serial.println("Starting MMA8451");
    if (!mma8451.begin()) {
        Serial.println("Failed to start MMA8451!");
    } else {
        Serial.println("MMA8451 found!");
    }

    mma8451.setRange(MMA8451_RANGE_2_G);
    mma8451.setDataRate(mma8451_dataRate_t::MMA8451_DATARATE_50_HZ);

    // Set up motor driver(s)
    setupMotor(motorVertical);
    setupMotor(motorHorizontal);
}

void calibrateVertical() {
    const float ZERO_CAL = 0.2;
    int targetVelocity;

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
            targetVelocity = TIC_SPEED_VERYSLOW;
            delay(50);
        } else {
            targetVelocity = TIC_SPEED_DEFAULT;
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
}

void parseCommand(String input) {
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
        if (arg1.isEmpty()) {
            Serial.println("ERR");
            return;
        }

        int position = arg1.toFloat();
        if (abs(position) > 90) {
            Serial.println("ERR");
            return;
        }

        motorVertical.setTargetPosition(position * float(TIC_STEPS_PER_DEGREE));
    } else if (command == "DHOR") {
        String arg1 = input.substring(indicies[0], indicies[1]);
        arg1.trim();

        if (arg1.isEmpty()) {
            Serial.println("ERR");
            return;
        }

        int position = arg1.toFloat();
        if (abs(position) > 180) {
            Serial.println("ERR");
            return;
        }

        motorHorizontal.setTargetPosition(position * float(TIC_STEPS_PER_DEGREE));
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
        signed char byte = Serial.read();
        if (byte == '\n') {
            // The command string has been terminated
            commandString += ' ';
            parseCommand(commandString);
            commandString.clear();
        } else if (byte == '\b' && commandString.length() != 0) {
            // Backspace, delete the last character in the string.
            commandString.remove(commandString.length() - 1);
        } else if (byte != -1) {
            // This is a regular valid character, add it to the string
            commandString += byte;
        }
    }

    delay(10);
}
