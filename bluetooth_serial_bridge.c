#include <Servo.h>
#include <SoftwareSerial.h>

// ===== Servo objects =====
// Create Servo objects for each joint of the robotic arm
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// ===== Bluetooth HC-05 =====
// Set up SoftwareSerial on pins 10 (RX) and 11 (TX) for HC-05
SoftwareSerial BT(10, 11);  // RX, TX

// ===== Servo angle limits =====
// Ensures servos do not move outside safe range
int minAngle = 0;
int maxAngle = 180;

// ===== Input buffer =====
// Stores incoming serial data line by line
String inputString = "";

void setup() {
  // Initialize USB serial
  Serial.begin(115200);
  // Initialize Bluetooth serial
  BT.begin(9600);   // HC-05 default baud rate

  // Attach each servo to its corresponding pin
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
  servo6.attach(11);

  // Set all servos to initial "neutral" position (90°)
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  servo6.write(90);

  // Print startup messages to Serial Monitor and Bluetooth
  Serial.println("Arduino ready. Waiting for 6 angles...");
  BT.println("Bluetooth ready. Waiting for 6 angles...");
}

void loop() {
  // ===== Read data from USB =====
  if (Serial.available()) {
    char c = Serial.read();  // Read one character at a time
    if (c == '\n') {         // End of line received
      processInput(inputString);  // Process the full line
      inputString = "";           // Clear buffer for next line
    } else {
      inputString += c;           // Append character to buffer
    }
  }

  // ===== Read data from Bluetooth =====
  if (BT.available()) {
    char c = BT.read();      // Read one character at a time
    if (c == '\n') {         // End of line received
      processInput(inputString);  // Process the full line
      inputString = "";           // Clear buffer for next line
    } else {
      inputString += c;           // Append character to buffer
    }
  }
}

// ===== Process received serial line =====
void processInput(String line) {
  line.trim(); // Remove leading/trailing whitespace
  if (line.length() == 0) return;  // Ignore empty lines

  // Array to store 6 angles for the servos
  int angles[6] = {90,90,90,90,90,90};
  int index = 0;

  // ===== Split line by commas =====
  // Example input: "90,120,45,60,180,30"
  int lastComma = -1;
  for (int i = 0; i < line.length(); i++) {
    // Check for comma or end of line
    if (line[i] == ',' || i == line.length() - 1) {
      int start = lastComma + 1;
      int end = (i == line.length() - 1) ? i+1 : i;
      String numStr = line.substring(start, end); // Extract substring
      numStr.trim();                               // Remove whitespace
      angles[index] = constrain(numStr.toInt(), minAngle, maxAngle); // Convert to int and constrain
      index++;
      lastComma = i;
      if (index >= 6) break; // Only 6 angles expected
    }
  }

  // ===== Move Servos =====
  servo1.write(angles[0]);
  servo2.write(angles[1]);
  servo3.write(angles[2]);
  servo4.write(angles[3]);
  servo5.write(angles[4]);
  servo6.write(angles[5]);

  // ===== Debug Output =====
  // Print the angles to Serial Monitor
  Serial.print("Moved to: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(angles[i]);
    if (i < 5) Serial.print(",");
  }
  Serial.println();

  // Print the angles back to Bluetooth
  BT.print("Moved to: ");
  for (int i = 0; i < 6; i++) {
    BT.print(angles[i]);
    if (i < 5) BT.print(",");
  }
  BT.println();
}
