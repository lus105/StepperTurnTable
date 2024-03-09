#include <Wire.h>
#include <AS5600.h>
#include <FastLED.h>
#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepperMotor(1, 2, 5);  // (Type of driver: with 2 pins, STEP, DIR)
int stepperMotorMaxSpeed = 3000;
int stepperMotorAcceleration = 6000;
int stepperMotorSpeed = 3000;
int stepperMotorCalibrationSpeed = 1000;
bool motorStarted = false;

#define NUM_LEDS 16
#define LED_PIN 3
CRGB leds[NUM_LEDS];

enum mode { SINGLERUN,
            FREERUN,
            POSITION };
#define STEPCOUNT 1600
int positions = 6;
int currentPositionIndex = 0;
float reduction = 20.f / 36.f;
mode currentMode = mode::FREERUN;


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
        break;
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
  PowerLED(true);
  if (stepperMotor.isRunning()) {
    Serial.println("Motor stop");
    stepperMotor.stop();
    delay(500);
  }
  // 3200 steps resolution
  static const float allowedError = 0.2f;
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  float angularDifference = currentAngle - desiredPositionAngle;
  if (angularDifference > 180.f) {
    angularDifference = 360.f - angularDifference;
  }
  if (abs(angularDifference) > allowedError) {
    while (true) {
      if (abs(angularDifference) < allowedError) {
        Serial.println("Calibration angular difference: " + String(angularDifference) + ", position is good enough, calibration done!");
        stepperMotor.setCurrentPosition(0);
        break;
      } else {
        Serial.println("Calibration angular difference: " + String(angularDifference) + +", " + String(stepperMotor.currentPosition()) + ", searching for position...");
      }
      stepperMotor.setSpeed(stepperMotorCalibrationSpeed);
      stepperMotor.runSpeed();
      // calculate difference between current and desired
      currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
      angularDifference = currentAngle - desiredPositionAngle;
      if (angularDifference > 180.f) {
        angularDifference = 360.f - angularDifference;
      }
    }
  }
  PowerLED(false);
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
  stepperMotor.setSpeed(stepperMotorSpeed);

  // setup leds
  FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);

  // Calibrate absolute zero position
  calibrateZeroPosition();

  // delay a bit before start
  delay(1000);
}

void ExecuteCommand(String command) {
  // extract action and value, or just basic command as start or stop
  if (command.equals("stop")) {
    Serial.println("stoping");
    motorStarted = false;
    stepperMotor.stop();
  } else if (command.equals("start")) {
    motorStarted = true;
    Serial.println("starting");
    currentMode = mode::FREERUN;
  } else if (command.equals("calibrate")) {
    calibrateZeroPosition();
  } else if (command.equals("position")) {
    motorStarted = true;
    currentMode = mode::POSITION;
    calibrateZeroPosition();
    currentPositionIndex = 0;
    Serial.println("positioning");
  } else if (command.equals("singlerun")) {
    currentMode = mode::SINGLERUN;
    calibrateZeroPosition();
    motorStarted = true;
  }
}

void PowerLED(bool on)
{
  if (on)
  {
    fill_solid(leds, NUM_LEDS, CRGB::White); // Turn all LEDs white
    FastLED.show();
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black); // Turn off all LEDs
    FastLED.show();
  }
}

void TriggerCamera()
{
  delay(1000);
}

int GetPositionAfterReduction(int desiredPosition, float reductionCoef = reduction)
{
  float endPosition = (float)desiredPosition / reductionCoef;
  return (int)endPosition;
}

void loop() {
  // create command variable once
  static String command = "";

  // If command is passed over Serial, try to apply it
  if (readCommand(command)) {
    Serial.println("Received command: " + command);
    ExecuteCommand(command);
  }

  if (motorStarted) {
    if (currentMode == mode::FREERUN) {
      stepperMotor.setSpeed(stepperMotorSpeed);
      stepperMotor.runSpeed();
    } else if (currentMode == mode::POSITION) {
      if (currentPositionIndex == 0) {
        stepperMotor.setCurrentPosition(0);
      }

      int newPositionStep = int(currentPositionIndex * STEPCOUNT / positions);
      newPositionStep = GetPositionAfterReduction(newPositionStep);
      Serial.println(newPositionStep);
      stepperMotor.moveTo(newPositionStep);
      stepperMotor.runToPosition();
      currentPositionIndex++;
      if (currentPositionIndex % (positions - 1) == 0) {
        currentPositionIndex = 0;
      }
      PowerLED(true);
      delay(500);
      TriggerCamera();
      PowerLED(false);
    } else if (currentMode == mode::SINGLERUN) {
      stepperMotor.setCurrentPosition(0);
      // move full circle, but consider reduction
      int position = GetPositionAfterReduction(STEPCOUNT);
      stepperMotor.moveTo(position); // full circle
      stepperMotor.runToPosition();
      // reset to different mode and stop motor
      currentMode = mode::FREERUN;
      motorStarted = false;
    }
  }
  // Read encoder
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  // Step the motor with a constant speed previously set by setSpeed();
  static unsigned long timer = 0;
  unsigned long interval = 1000;
  if (millis() - timer >= interval) {
    timer = millis();
    Serial.println("Stepper motor pos: " + String(stepperMotor.currentPosition()) + " encoder angle: " + String(currentAngle));
  }
}
