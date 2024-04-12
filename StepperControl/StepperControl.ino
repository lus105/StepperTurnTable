#include <Wire.h>
#include <AS5600.h>
#include <FastLED.h>
#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepperMotor(1, 4, 3);  // (Type of driver: with 2 pins, STEP, DIR)
int stepperMotorMaxSpeed = 2000;
int stepperMotorAcceleration = 1000;
int stepperMotorSpeed = 2000;
int stepperMotorCalibrationSpeed = 1000;
bool motorStarted = false;
const int stepsPerDegree = 8;

#define NUM_LEDS 16
#define LED_PIN 2
#define CAMERA_PIN 5
CRGB leds[NUM_LEDS];

enum mode { SINGLERUN,
            FREERUN,
            POSITION };
#define STEPCOUNT 1600
int positions = 6;
int currentPositionIndex = 0;
float reduction = 20.f / 36.f;
mode currentMode = mode::FREERUN;
bool ledPowered = false;

// Define ams5600-based encoder
AS5600 as5600;
#define retryTimeForMagnetSearch 1000

// Additional constants
// Global variable to store the last command time
unsigned long lastCommandTime = 0;
const unsigned long commandDebounceTime = 500;

// AS5600 encoder needs to search for the magnet first before the launch
// Loop multiple times try to find magnet
bool isMagnetOnEncoderDetected(size_t retry_times = retryTimeForMagnetSearch)
{
  bool magnetDetected = false;
  for (size_t i = 0; i < retry_times; i++)
  {
    if (as5600.detectMagnet())
    {
      Serial.print("Current Magnitude: ");
      Serial.println(as5600.readMagnitude());
      magnetDetected = true;
      break;
    }
    else
    {
      if (as5600.magnetTooWeak())
      {
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
void calibrateZeroPosition(float desiredPositionAngle = 0.f)
{
  Serial.println("Calibration...");
  PowerLED(200, 10, 10);
  if (stepperMotor.isRunning())
  {
    Serial.println("Motor stop");
    stepperMotor.stop();
    delay(500);
  }
  // 3200 steps resolution
  static const float allowedError = 0.2f;
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  float angularDifference = currentAngle - desiredPositionAngle;
  if (angularDifference > 180.f)
  {
    angularDifference = 360.f - angularDifference;
  }
  if (abs(angularDifference) > allowedError)
  {
    while (true)
    {
      if (abs(angularDifference) < allowedError)
      {
        Serial.println("Calibration angular difference: " + String(angularDifference) + ", position is good enough, calibration done!");
        stepperMotor.setCurrentPosition(0);
        PowerLED(20, 225, 20);
        delay(3000);
        break;
      }
      else
      {
        Serial.println("Calibration angular difference: " + String(angularDifference) + +", " + String(stepperMotor.currentPosition()) + ", searching for position...");
      }
      stepperMotor.setSpeed(stepperMotorCalibrationSpeed);
      stepperMotor.runSpeed();
      // calculate difference between current and desired
      currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
      angularDifference = currentAngle - desiredPositionAngle;
      if (angularDifference > 180.f)
      {
        angularDifference = 360.f - angularDifference;
      }
    }
  }
  PowerLED(false);
}

// Function to read command from serial
bool readCommand(String& command)
{
  if (!Serial.available() || millis() - lastCommandTime < commandDebounceTime)
  {
    command = "";
    return false;
  }
  command = Serial.readStringUntil('\n');
  lastCommandTime = millis();
  return true;
}

void setup()
{
  // Setup for printing into serial and reading from serial
  Serial.begin(115200);

  // initialize i2c interface for ams5600 encoder
  Wire.begin();

  // delay a bit before start
  delay(1000);

  // Run magnet detection on the encoder routine
  bool encoderMagnetDetected = isMagnetOnEncoderDetected();
  if (encoderMagnetDetected)
  {
    Serial.println("Magnet detected, run the application!");
  }
  else
  {
    Serial.println("Magnet not found, encoder is useless!");
  }

  // Set maximum speed value for the stepper
  stepperMotor.setMaxSpeed(stepperMotorMaxSpeed);
  stepperMotor.setAcceleration(stepperMotorAcceleration);
  stepperMotor.setSpeed(stepperMotorSpeed);

  // setup leds
  FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
  pinMode(CAMERA_PIN, OUTPUT);
  digitalWrite(CAMERA_PIN, LOW);
  delay(1000);

  // Calibrate absolute zero position
  calibrateZeroPosition();

  // delay a bit before start
  delay(1000);
}

void ExecuteCommand(String command)
{
  // extract action and value, or just basic command as start or stop
  if (command.equals("stop"))
  {
    Serial.println("stoping");
    motorStarted = false;
    stepperMotor.stop();
  }
  else if (command.equals("start"))
  {
    motorStarted = true;
    Serial.println("starting");
    currentMode = mode::FREERUN;
  }
  else if
  (command.equals("calibrate"))
  {
    calibrateZeroPosition();
  }
  else if (command.equals("position"))
  {
    motorStarted = true;
    currentMode = mode::POSITION;
    calibrateZeroPosition();
    currentPositionIndex = 0;
    Serial.println("positioning");
  }
  else if (command.equals("singlerun"))
  {
    currentMode = mode::SINGLERUN;
    calibrateZeroPosition();
    motorStarted = true;
  }
  else if (command.equals("powerled"))
  {
    ledPowered = !ledPowered;
    PowerLED(ledPowered);
  }
}

// white (on), black - off
void PowerLED(bool on)
{
  if (on && ledPowered)
  {
    fill_solid(leds, NUM_LEDS, CRGB::White); // Turn all LEDs white
    FastLED.show();
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Black); // Turn off all LEDs
    FastLED.clear();
    FastLED.show();
  }
}

// defined color
void PowerLED(byte R, byte G, byte B)
{
    fill_solid(leds, NUM_LEDS, CRGB(G, R, B));
    FastLED.show();
}

void TriggerCamera()
{
  digitalWrite(CAMERA_PIN, HIGH);
  delay(100);
  digitalWrite(CAMERA_PIN, LOW);
}

int GetPositionAfterReduction(int desiredPosition, float reductionCoef = reduction)
{
  float endPosition = (float)desiredPosition / reductionCoef;
  return (int)endPosition;
}

// Function to move to a desired angle with error compensation
void moveToDesiredAngle(float desiredAngle)
{
  const float allowedError = 0.2f; // Acceptable error in degrees
  float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  float error = currentAngle - desiredAngle;

  if (error > 180) error -= 360;
  else if (error < -180) error += 360;

  float speedReductionFactor;

  while (abs(error) > allowedError)
  {
    // Calculate steps needed to correct error, adjust this calculation based on your setup
    int correctionSteps = error * stepsPerDegree; // stepsPerDegree needs to be defined based on your motor and reduction setup
    speedReductionFactor = max(0.1, 1 - (abs(error) / 180));
    stepperMotor.setSpeed(stepperMotorSpeed * speedReductionFactor);
    stepperMotor.move(correctionSteps);
    while (stepperMotor.isRunning()) stepperMotor.run();

    // Re-measure the angle to see if error is within acceptable bounds
    currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    error = currentAngle - desiredAngle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
  }
  stepperMotor.setSpeed(stepperMotorSpeed);
}

void loop()
{
  // create command variable once
  static String command = "";

  // If command is passed over Serial, try to apply it
  if (readCommand(command))
  {
    Serial.println("Received command: " + command);
    ExecuteCommand(command);
  }

  if (motorStarted)
  {
    if (currentMode == mode::FREERUN)
    {
      stepperMotor.setSpeed(stepperMotorSpeed);
      stepperMotor.runSpeed();
    }
    else if (currentMode == mode::POSITION)
    {
      int newPositionStep = int(currentPositionIndex * STEPCOUNT / positions);
      if (currentPositionIndex - 1 == positions)
      {
        newPositionStep = GetPositionAfterReduction(STEPCOUNT);
      }
      else
      {
        newPositionStep = GetPositionAfterReduction(newPositionStep);
      }
      Serial.println(newPositionStep);
      stepperMotor.moveTo(newPositionStep);
      while (stepperMotor.distanceToGo() != 0)
      {
        stepperMotor.run();
      }

      currentPositionIndex++;
      if (currentPositionIndex % positions == 0)
      {
        currentPositionIndex = 1;
        stepperMotor.setCurrentPosition(0);
      }
      PowerLED(true);
      delay(1000);
      TriggerCamera();
      PowerLED(false);
    }
    else if (currentMode == mode::SINGLERUN)
    {
      stepperMotor.setCurrentPosition(0);
      // move full circle, but consider reduction
      int position = GetPositionAfterReduction(STEPCOUNT);
      Serial.println(position);
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
  if (millis() - timer >= interval)
  {
    timer = millis();
    //Serial.println("Stepper motor pos: " + String(stepperMotor.currentPosition()) + " encoder angle: " + String(currentAngle));
  }
}
