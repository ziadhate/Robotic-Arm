#include <Servo.h>
#include <math.h>

// ==================== Servo Objects ====================
// Create servo objects for each joint in the robot arm
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo; // Servo for the gripper (optional)

// ==================== Arm Dimensions ====================
// Lengths of the robotic arm segments in centimeters
float L1 = 16.0;   // Shoulder → Elbow segment
float L2 = 11.5;   // Elbow → Wrist segment

// ==================== Target Coordinates ====================
// These will hold received X, Y, Z values from the computer (Python)
float X = 0, Y = 0, Z = 0;

// ==================== Z-Height Settings ====================
// Fixed heights for picking and placing the object
float Z_pick = 5.0;   // Height at which the robot picks the object
float Z_place = 15.0; // Height for lifting the object after gripping

// ==================== Setup ====================
void setup() {
  Serial.begin(115200); // Start serial communication with Python

  // Attach servos to their pins
  baseServo.attach(3);
  shoulderServo.attach(5);
  elbowServo.attach(6);
  gripperServo.attach(9);

  // Set initial neutral servo positions
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  gripperServo.write(0);  // Start with gripper open

  Serial.println("Arduino Ready");
}

// ==================== Inverse Kinematics Function ====================
// This function calculates servo angles needed to reach point (x, y, z)
void moveArmTo(float x, float y, float z) {

  // ----- 1. Base Rotation Angle (B) -----
  // Rotates around the vertical axis to align with target
  float B = atan2(y, x) * 180.0 / M_PI;

  // ----- 2. XY-plane distance -----
  float d_xy = sqrt(x*x + y*y);

  // Vertical height
  float dz = z;

  // Total straight-line distance from shoulder joint to target
  float d_total = sqrt(d_xy*d_xy + dz*dz);

  // ----- 3. Elbow Angle Calculation (Law of Cosines) -----
  float cosE = (d_total*d_total - L1*L1 - L2*L2) / (2*L1*L2);

  // Constrain to valid cosine range to avoid NAN
  cosE = constrain(cosE, -1.0, 1.0);

  float E = acos(cosE) * 180.0 / M_PI;

  // ----- 4. Shoulder Angle Calculation -----
  float S = atan2(dz, d_xy) * 180.0 / M_PI -
            atan2(L2 * sin(E * M_PI/180.0),
                  L1 + L2 * cos(E * M_PI/180.0)) * 180.0 / M_PI;

  // ----- 5. Convert to Servo Angles -----
  int baseAngle = map(B, -90, 90, 0, 180); // Convert -90..90 to servo range
  int shoulderAngle = (int)(S + 90);       // Offset to match servo zero
  int elbowAngle = 180 - (int)E;           // Reverse elbow orientation

  // ----- 6. Move Servos -----
  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);

  // Debug output in serial monitor
  Serial.print("Moving -> B:");
  Serial.print(baseAngle);
  Serial.print(" S:");
  Serial.print(shoulderAngle);
  Serial.print(" E:");
  Serial.println(elbowAngle);
}


// ==================== Main Loop ====================
void loop() {

  // Check for incoming data from Python
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    if (data.length() > 0) {

      // Extract X/Y/Z from string "X10.3 Y5.5 Z0"
      int ix = data.indexOf('X');
      int iy = data.indexOf('Y');
      int iz = data.indexOf('Z');

      // Only continue if all coordinates exist
      if (ix != -1 && iy != -1 && iz != -1) {

        X = data.substring(ix + 1, iy).toFloat();
        Y = data.substring(iy + 1, iz).toFloat();
        Z = data.substring(iz + 1).toFloat();

        // Print received coordiantes
        Serial.print("Received -> X:");
        Serial.print(X);
        Serial.print(" Y:");
        Serial.print(Y);
        Serial.print(" Z:");
        Serial.println(Z);

        // ==================== Pick & Place Sequence ====================

        // ---- Step 1: Move above object at picking height ----
        moveArmTo(X, Y, Z_pick);
        delay(1500);

        // Close gripper to grab the object
        gripperServo.write(90); // Change 90 if your gripper closes differently
        delay(500);

        // ---- Step 2: Lift object ----
        moveArmTo(X, Y, Z_place);
        delay(1500);

        // ---- Step 3: Move back to home position ----
        moveArmTo(0, 0, Z_place);
        delay(1000);

        // Open gripper to release object
        gripperServo.write(0);
        delay(500);
      }
    }
  }
}
