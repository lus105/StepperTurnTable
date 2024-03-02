#include <Wire.h>
#include <AS5600.h>

#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepperMotor(1, 2, 5);  // (Type of driver: with 2 pins, STEP, DIR)
int stepperMotorMaxSpeed = 5000;
int stepperMotorAcceleration = 100;
#define calibrationSpeed 500

// Define ams5600-based encoder
AS5600 as5600;
#define retryTimeForMagnetSearch 1000

// AS5600 encoder needs to search for the magnet first before the launch
// Loop multiple times try to find magnet
bool isMagnetOnEncoderDetected(size_t retry_times = retryTimeForMagnetSearch) {
  bool magnetDetected = false;
  for (size_t i = 0; i < retry_times; i++) {
    if (as5600.detectMagnet()) {
      Serial.print("Current Magnitude: ");
      Serial.println(as5600.readMagnitude());
      magnetDetected = true;
      break;
    } else {
      if (as5600.magnetTooWeak()) {
        Serial.println("Magnet too weak..");
      }
      Serial.println("Searching...");
    }
    delay(1000);
  }
  return magnetDetected;
}

// Calibrate zero (absolute) position with encoder
void calibrateZeroPosition(float desiredPositionAngle = 0.f) {
  Serial.println("Calibration...");
  if (stepperMotor.isRunning()) {
    stepperMotor.stop();
    delay(500);
  }
  // TODO: maybe limit in the time?

  // reduce speed for calibration and save the last speed and try
  // also, there can be allowed error, since the motor with control cannot deliver the exact position
  // try to define it - for NEMA 17 it is 200 steps / revolution (1.8 degrees)
  // lets make allowed error to be half of it - 0.9 degrees or smaller
  static const float allowedError = 0.9f;
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  float angularDifference = currentAngle - desiredPositionAngle;
  if (angularDifference > 180.f)
  {
    angularDifference = 360.f - angularDifference;
  }
  if (abs(angularDifference) > allowedError) {
    // reduce speed for calibration
    stepperMotor.setMaxSpeed(calibrationSpeed);
    stepperMotor.setSpeed(calibrationSpeed);
    while (true) {
      if (abs(angularDifference) < allowedError) {
        stepperMotor.stop();
        Serial.println("Calibration angular difference: " + String(angularDifference) + ", position is good enough, calibration done!");
        stepperMotor.setCurrentPosition(0);
        break;
      } else {
        Serial.println("Calibration angular difference: " + String(angularDifference) + ", searching for position...");
        stepperMotor.runSpeed();
      }
      // calculate difference between current and desired
      currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
      angularDifference = currentAngle - desiredPositionAngle;
      if (angularDifference > 180.f)
      {
        angularDifference = 360.f - angularDifference;
      }
    }
    // reset speed
    stepperMotor.setMaxSpeed(stepperMotorMaxSpeed);
  }
}

// Function to read command from serial
bool readCommand(String& command) {
  if (!Serial.available()) {
    command = "";
    return false;
  }
  command = Serial.readStringUntil('\n');
  return true;
}

void setup() {
  // Setup for printing into serial and reading from serial
  Serial.begin(115200);

  // initialize i2c interface for ams5600 encoder
  Wire.begin();

  // delay a bit before start
  delay(1000);

  // Run magnet detection on the encoder routine
  bool encoderMagnetDetected = isMagnetOnEncoderDetected();
  if (encoderMagnetDetected) {
    Serial.println("Magnet detected, run the application!");
  } else {
    Serial.println("Magnet not found, encoder is useless!");
  }

  // Set maximum speed value for the stepper
  stepperMotor.setMaxSpeed(stepperMotorMaxSpeed);
  stepperMotor.setAcceleration(stepperMotorAcceleration);

  // Calibrate absolute zero position
  calibrateZeroPosition();

  // delay a bit before start
  delay(1000);
}

void loop() {
  // create command variable once
  static String command = "";

  // If command is passed over Serial, try to apply it
  if (readCommand(command)) {
    Serial.println("Received command: " + command);
  }

  // Read encoder
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;

  // Step the motor with a constant speed previously set by setSpeed();
  // stepperMotor.runSpeed();
  static unsigned long timer = 0;
  unsigned long interval = 100;
  if (millis() - timer >= interval) {
    timer = millis();
    Serial.println("Stepper motor pos: " + String(stepperMotor.currentPosition()) + " encoder angle: " + String(currentAngle));
  }
}
