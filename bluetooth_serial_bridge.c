#include <Servo.h>           // Library for controlling servo motors
#include <SoftwareSerial.h>  // Library to create additional Serial (for Bluetooth, etc.)

// ====== SERVOS ======
Servo s1;  // Servo for joint 1
Servo s2;  // Servo for joint 2
Servo s3;  // Servo for joint 3
Servo s4;  // Servo for joint 4
Servo s5;  // Servo for joint 5
Servo s6;  // Servo for joint 6

// ====== VARIABLES ======
int angles[6];     // Array to store the received angles for each servo
String input = ""; // String to accumulate incoming data before parsing

// ====== Bluetooth RX/TX ======
#define RX_PIN 2      // Pin for receiving data from Bluetooth
#define TX_PIN 12     // Pin for sending data to Bluetooth (different from 11 to avoid conflict with Servo 6)
SoftwareSerial BT(RX_PIN, TX_PIN); // Create a new SoftwareSerial for Bluetooth

void setup() {
  Serial.begin(9600); // Initialize USB Serial communication (optional)
  BT.begin(9600);     // Initialize Bluetooth communication at 9600 baud

  // Attach servos to PWM pins
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  s4.attach(9);
  s5.attach(10);
  s6.attach(11);

  // Indicate system is ready
  Serial.println("READY");  
  BT.println("READY");
}

void loop() {
  // ===== Read data from USB Serial =====
  while (Serial.available()) {   // While data is available
    char c = Serial.read();      // Read one character
    if (c == '\n') {             // If newline character is received
      parseAngles(input);        // Parse the string into angles
      input = "";                // Reset the input string for the next message
    } else {
      input += c;                // Append character to the input string
    }
  }

  // ===== Read data from Bluetooth Serial =====
  while (BT.available()) {       // While data is available from Bluetooth
    char c = BT.read();          // Read one character
    if (c == '\n') {             // If newline character is received
      parseAngles(input);        // Parse the string into angles
      input = "";                // Reset input string
    } else {
      input += c;                // Append character to the input string
    }
  }
}

// ====== Function to parse the string into angles ======
void parseAngles(String line) {
  int idx = 0;   // Index of the next comma
  int last = 0;  // Start position of the current number

  for (int i = 0; i < 6; i++) {           // Loop for all 6 servos
    idx = line.indexOf(',', last);         // Find the next comma
    if (idx == -1 && i < 5) return;       // Invalid data if comma is missing before the last number
    if (i == 5) {
      angles[i] = line.substring(last).toInt(); // Last number (no comma at the end)
    } else {
      angles[i] = line.substring(last, idx).toInt(); // Extract number between commas
    }
    last = idx + 1;                        // Update start position for the next number
  }

  // ====== Move the servos ======
  s1.write(angles[0]);
  s2.write(angles[1]);
  s3.write(angles[2]);
  s4.write(angles[3]);
  s5.write(angles[4]);
  s6.write(angles[5]);

  // ====== Send feedback to both USB and Bluetooth ======
  Serial.print("OK: ");  Serial.println(line);  
  BT.print("OK: ");      BT.println(line);      
}
