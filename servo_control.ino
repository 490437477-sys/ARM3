/*
 * Servo Arm Control System
 * Main Controller: Arduino UNO R3
 * 
 * Hardware:
 * - 2x Dual-axis Joystick Modules
 * - 5x MG995 Servo Motors
 * - No LED
 * 
 * Control Methods:
 * 1. Joystick Control - Physical joysticks
 * 2. Serial Commands - Arduino IDE Serial Monitor
 * 3. Python GUI - Computer interface
 */

#include <Servo.h>

// ===== PIN DEFINITIONS =====
// Joystick 1 pins
const int joy1XPin = A0;  // Joystick 1 X-axis
const int joy1YPin = A1;  // Joystick 1 Y-axis
const int joy1BtnPin = 2; // Joystick 1 Button

// Joystick 2 pins
const int joy2XPin = A3;  // Joystick 2 X-axis
const int joy2YPin = A2;  // Joystick 2 Y-axis
const int joy2BtnPin = 4; // Joystick 2 Button

// Servo pins
const int servoPins[] = {5, 9, 10, 11, 7};  // Servo 0-4

// ===== CONTROL PARAMETERS =====
const int deadZone = 50;      // Joystick dead zone
const int moveDelay = 50;      // Movement delay (ms)
const int btnDebounce = 50;   // Button debounce (ms)
const int btnStep = 2;         // Button step per press

// ===== VARIABLES =====
Servo servos[5];
int angles[] = {90, 90, 90, 90, 90};     // Current angles
int targets[] = {90, 90, 90, 90, 90};    // Target angles

unsigned long lastMove = 0;
unsigned long lastBtn1 = 0;
unsigned long lastBtn2 = 0;

String inputBuffer = "";

// ===== SETUP =====
void setup() {
  // Configure pins
  pinMode(joy1BtnPin, INPUT_PULLUP);
  pinMode(joy2BtnPin, INPUT_PULLUP);

  // Initialize servos
  for (int i = 0; i < 5; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(angles[i]);
    delay(50);
  }

  // Initialize serial
  Serial.begin(9600);
  while (!Serial) delay(10);

  printHelp();
  printStatus();
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long now = millis();

  // Movement timing
  if (now - lastMove < moveDelay) {
    delay(moveDelay - (now - lastMove));
    return;
  }
  lastMove = now;

  // Process serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        Serial.print("> "); Serial.println(inputBuffer);
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // Smooth movement to targets
  for (int i = 0; i < 5; i++) {
    if (angles[i] < targets[i]) angles[i]++;
    else if (angles[i] > targets[i]) angles[i]--;
    servos[i].write(angles[i]);
  }

  // Read joysticks
  int j1x = analogRead(joy1XPin) - 512;
  int j1y = analogRead(joy1YPin) - 512;
  int j2x = analogRead(joy2XPin) - 512;
  int j2y = analogRead(joy2YPin) - 512;

  // Joystick 1: X->Servo0, Y->Servo1
  if (j1x < -deadZone) { targets[0] = constrain(targets[0] - 1, 0, 180); angles[0] = targets[0]; }
  if (j1x > deadZone) { targets[0] = constrain(targets[0] + 1, 0, 180); angles[0] = targets[0]; }
  if (j1y < -deadZone) { targets[1] = constrain(targets[1] - 1, 0, 180); angles[1] = targets[1]; }
  if (j1y > deadZone) { targets[1] = constrain(targets[1] + 1, 0, 180); angles[1] = targets[1]; }

  // Joystick 2: X->Servo2, Y->Servo3
  if (j2x < -deadZone) { targets[2] = constrain(targets[2] - 1, 0, 180); angles[2] = targets[2]; }
  if (j2x > deadZone) { targets[2] = constrain(targets[2] + 1, 0, 180); angles[2] = targets[2]; }
  if (j2y < -deadZone) { targets[3] = constrain(targets[3] - 1, 0, 180); angles[3] = targets[3]; }
  if (j2y > deadZone) { targets[3] = constrain(targets[3] + 1, 0, 180); angles[3] = targets[3]; }

  // Button controls -> Servo 4
  if (digitalRead(joy1BtnPin) == LOW && now - lastBtn1 > btnDebounce) {
    targets[4] = constrain(targets[4] + btnStep, 0, 90);
    angles[4] = targets[4];
    lastBtn1 = now;
  }
  if (digitalRead(joy2BtnPin) == LOW && now - lastBtn2 > btnDebounce) {
    targets[4] = constrain(targets[4] - btnStep, 0, 90);
    angles[4] = targets[4];
    lastBtn2 = now;
  }
}

// ===== SERIAL COMMANDS =====
void printHelp() {
  Serial.println("=== Servo Arm Control ===");
  Serial.println("Commands:");
  Serial.println("  0 90    - Set Servo0 to 90 deg");
  Serial.println("  1 45    - Set Servo1 to 45 deg");
  Serial.println("  2 90    - Set Servo2 to 90 deg");
  Serial.println("  3 90    - Set Servo3 to 90 deg");
  Serial.println("  4 45    - Set Servo4 to 45 deg (max 90)");
  Serial.println("  90      - Set ALL servos to 90 deg");
  Serial.println("  90 90 90 90 90 - Batch set");
  Serial.println("  s       - Show status");
  Serial.println("  h       - Show help");
  Serial.println("========================");
}

void printStatus() {
  Serial.print("S:");
  for (int i = 0; i < 5; i++) {
    Serial.print(angles[i]);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "h" || cmd == "help") { printHelp(); return; }
  if (cmd == "s" || cmd == "status") { printStatus(); return; }

  int values[5] = {0, 0, 0, 0, 0};
  int count = 0;
  int start = 0;

  for (int i = 0; i <= cmd.length() && count < 5; i++) {
    if (cmd.charAt(i) == ' ' || i == cmd.length()) {
      values[count++] = cmd.substring(start, i).toInt();
      start = i + 1;
    }
  }

  if (count == 5) {
    for (int i = 0; i < 5; i++) {
      int maxAngle = (i == 4) ? 90 : 180;
      targets[i] = constrain(values[i], 0, maxAngle);
    }
    Serial.print("OK:");
    for (int i = 0; i < 5; i++) { Serial.print(targets[i]); if (i < 4) Serial.print(","); }
    Serial.println();
    return;
  }

  if (count == 1 && values[0] >= 0 && values[0] <= 180) {
    for (int i = 0; i < 5; i++) targets[i] = values[0];
    Serial.print("ALL:"); Serial.println(values[0]);
    return;
  }

  if (count == 2) {
    int servo = values[0];
    int angle = values[1];
    if (servo >= 0 && servo <= 4 && angle >= 0) {
      int maxAngle = (servo == 4) ? 90 : 180;
      targets[servo] = constrain(angle, 0, maxAngle);
      Serial.print("S"); Serial.print(servo); Serial.print(":"); Serial.println(targets[servo]);
      return;
    }
  }

  Serial.println("?");
}
