#include <Arduino.h>
#include <Wire.h>
#include <Tic.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>

// Altitude and Azimuth
TicI2C motorH(14);
TicI2C motorV(15);

Adafruit_MMA8451 mma = Adafruit_MMA8451();
float x, y, z;
double roll = 0.00, pitch = 0.00;
double targetPitchSteps = 0.0;
double targetAzimuth = 0.0;
int stepsPerDegree = 126.0;

unsigned long startTimeMS = 0;
float startPitch = 0.0;
const int RANGE = 100;

// x: -263	y: 194	z: −142
// x: -33   y: -24  z: -18

const int OFFSET_X = 263;
const int OFFSET_Y = -194;
const int OFFSET_Z = 142;

const int SDA_PIN = 18;
const int SCL_PIN = 19;

void RP_calculate() {
  mma.read();
  x = float(mma.y - OFFSET_Y) / 4096;
  y = float(mma.x - OFFSET_X) / 4096;
  z = float(mma.z - OFFSET_Z) / 4096;

  roll = atan2(y , z) * 57.29577951;
  pitch = (atan2(-x, pow(y, 2) + pow(z, 2) ) * 57.29577951);
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

void calibrateVertical() {
  const float ZERO_CAL = 0.2;
  int targetVelocity = 2000000;

  // Find the zero position
  while (true) {
    double pitchSum = 0; 
    for (int i = 0; i < 20; i++) {
      RP_calculate();
      pitchSum += pitch;
      delay(20);
    }
    pitchSum /= 20;
    //Serial.println(pitchSum);

    // Prevents movement from erroring out
    motorV.resetCommandTimeout();

    if (abs(pitchSum - ZERO_CAL) < 3.0) {
      targetVelocity = 500000;
      delay(50);
    } else {
      targetVelocity = 7000000;
    }
    
    if (pitchSum <= -0.02) {
      motorV.setTargetVelocity(targetVelocity);
    } else if (pitchSum >= 0.02) {
      motorV.setTargetVelocity(-targetVelocity);
    } else {
      motorV.haltAndSetPosition(0);
      break;
    }
    delay(200);
    motorV.setTargetVelocity(0);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(100);

  Serial.println("Starting MMA8451");
  if (!mma.begin()) {
    Serial.println("Failed to start MMA8451!");
  } else {
    Serial.println("MMA8451 found!");
  }

  mma.setRange(MMA8451_RANGE_2_G);
  mma.setDataRate(mma8451_dataRate_t::MMA8451_DATARATE_50_HZ);

  // Set up motor driver(s)
  setupMotor(motorV);
  setupMotor(motorH);
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

    targetPitchSteps = position * float(stepsPerDegree);
    motorV.setTargetPosition(targetPitchSteps);
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

    targetPitchSteps = position * float(stepsPerDegree);
    motorH.setTargetPosition(targetPitchSteps);
  } else if (command == "CALV") {
    String arg1 = input.substring(indicies[0], indicies[1]);
    arg1.trim();
    if (arg1 == "SET") {
      motorV.haltAndSetPosition(0);
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

String builtString = "";

void loop() {
  // Prevents movement from erroring out
  motorH.resetCommandTimeout();
  motorV.resetCommandTimeout();

  if (Serial.available() > 0) {
    // Read in a string until a newline, not including the newline
    char byte = Serial.read();
    if (byte == '\n') {
      builtString += ' ';
      parseCommand(builtString);
      builtString.clear();
    } else if (byte == 8) {
      builtString.remove(builtString.length() - 1);
    } else if (byte != 0xFF) {
      builtString += byte;
    }
  }

  delay(10);
}
