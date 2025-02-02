#include <Tic.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>

// Altitude and Azimuth
TicI2C motorVertical(14);
TicI2C motorHorizontal(15);

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

void RP_calculate() {
  mma.read();
  x = float(mma.y - OFFSET_Y) / 4096;
  y = float(mma.x - OFFSET_X) / 4096;
  z = float(mma.z - OFFSET_Z) / 4096;

  roll = atan2(y , z) * 57.29577951;
  pitch = (atan2(-x, pow(y, 2) + pow(z, 2) ) * 57.29577951);
}

void setup() {
  Serial.begin(115200);

  while (!Serial) delay(10);

  Wire.begin();

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
  setupMotor(motorVertical);
  setupMotor(motorHorizontal);
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
    resetCommandTimeout(motorHorizontal);

    if (abs(pitchSum - ZERO_CAL) < 3.0) {
      targetVelocity = 500000;
      delay(50);
    } else {
      targetVelocity = 7000000;
    }
    
    if (pitchSum <= -0.02) {
      motorHorizontal.setTargetVelocity(targetVelocity);
    } else if (pitchSum >= 0.02) {
      motorHorizontal.setTargetVelocity(-targetVelocity);
    } else {
      motorHorizontal.haltAndSetPosition(0);
      break;
    }
    delay(200);
    motorHorizontal.setTargetVelocity(0);
  }

  Serial.println("Zeroing calibration finished!");
}

void resetCommandTimeout(TicI2C tic) {
  tic.resetCommandTimeout();
}

void loop() {
  // Prevents movement from erroring out
  motorVertical.resetCommandTimeout();
  motorHorizontal.resetCommandTimeout();

  if (Serial.available() > 0) {
    String str = Serial.readBytesUntil('\n')();
    str.trim();
    int position = str.toFloat();

    if (abs(position) > 90) {
      Serial.println("Position out of range 90° to -90°");
      return;
    }

    Serial.print("Travelling to ");
    Serial.print(position);
    Serial.println("°");

    targetPitchSteps = position * float(stepsPerDegree); //targetAzimuth was changed to targetPitchSteps by Manas to fix tilting. the variable was named wrongly
    motorHorizontal.setTargetPosition(targetPitchSteps);
  }

  delay(10);
}
