#include <Wire.h>
#include <AS5600.h>

#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepperMotor(1, 2, 5);  // (Type of driver: with 2 pins, STEP, DIR)

// Define ams5600-based encoder
AS5600 as5600;

// magnet detection state
bool encoderMagnetDetected = false;

bool isMagnetOnEncoderDetected(size_t retry_times = 5) {
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

void setup() {
  // Setup for printing into serial
  Serial.begin(115200);

  // initialize i2c interface for ams5600 encoder
  Wire.begin();

  // Run magnet detection on the encoder routine
  encoderMagnetDetected = isMagnetOnEncoderDetected();
  if (encoderMagnetDetected) {
    Serial.println("Magnet detected, run the application!");
  } else {
    Serial.println("Magnet not found, encoder is useless!");
  }

  // Set maximum speed value for the stepper
  stepperMotor.setMaxSpeed(5000);
  stepperMotor.setAcceleration(100);
  stepperMotor.setSpeed(1000);
}

void loop() {
  // Read encoder
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;

  // Step the motor with a constant speed previously set by setSpeed();
  stepperMotor.runSpeed();
  static unsigned long timer = 0;
  unsigned long interval = 100;
  if (millis() - timer >= interval) {
    timer = millis();
    Serial.println("Stepper motor pos: " + String(stepperMotor.currentPosition()) + " encoder angle: " + String(currentAngle));
  }
}