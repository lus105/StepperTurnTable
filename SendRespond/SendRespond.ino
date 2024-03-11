
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
  // delay a bit before start
  delay(1000);
}

void ExecuteCommand(String command) {
  // extract action and value, or just basic command as start or stop
  if (command.equals("stop")) {
    Serial.println("stoping...");
  } else if (command.equals("start")) {
    Serial.println("starting...");
  } else if (command.equals("calibrate")) {
     Serial.println("calibrating...");
  } else if (command.equals("position")) {
    Serial.println("positioning...");
  } else if (command.equals("singlerun")) {
    Serial.println("single running...");
  }
}

void loop() {
  // create command variable once
  static String command = "";

  // If command is passed over Serial, try to apply it
  if (readCommand(command)) {
    Serial.println("Received command: " + command);
    ExecuteCommand(command);
  }
  static unsigned long timer = 0;
  unsigned long interval = 1000;
  if (millis() - timer >= interval) {
    timer = millis();
    float time = timer / 1000.f;
    Serial.println("I am awake and alive for " + String(time) + " seconds!");
  }
}
