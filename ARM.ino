ARM:

#include <AFMotor.h>

// === Define Motors for Arm on the Second L293D Shield ===
AF_DCMotor baseMotor(1);    // M1 - Base Rotation
AF_DCMotor armMotor1(2);    // M2 - Arm Motor 1 (Forward/Backward)
AF_DCMotor gripperMotor(3); // M3 - Claw (Open/Close)
AF_DCMotor armMotor2(4);    // M4 - Arm Motor 2 (Forward/Backward)

void setup() {
  Serial.begin(9600);

  // Set speed for Arm Motors
  baseMotor.setSpeed(100);  // Slow down base rotation
  armMotor1.setSpeed(200);  
  gripperMotor.setSpeed(255);  // Max power to fully open gripper
  armMotor2.setSpeed(200);  

  stopAllMotors();
}

void loop() {
  Serial.println("Starting arm movement...");

  // Step 1: Move Arm Forward (Down)
  moveArmForward();
  delay(2000);
  
  // Step 2: Open and Close Gripper (Grab Weed)
  openGripperWide();
  delay(1500);  // Increased delay for wider opening
  closeGripper();
  delay(1000);

  // Step 3: Move Arm Back Up
  moveArmBackward();
  delay(2000);

  // Step 4: Rotate Base to Change Position
  rotateBase();
  delay(1500);  // Allowing more time for smooth rotation

  // Step 5: Open Gripper to Drop the Weed
  openGripperWide();
  delay(1500);
  
  // Step 6: Close Gripper Completely After Dropping
  closeGripper();
  delay(1000);

  // Step 7: Rotate Base Back to Original Position
  rotateBaseBack();
  delay(1500);

  // Step 8: Move Arm Back to Original Position
  moveArmForward();  // Moves back down slightly to reset
  delay(1000);
  moveArmBackward(); // Moves back up to fully reset
  delay(2000);

  Serial.println("Arm movement complete. Rover will move next.");
}

// ==================== ARM MOVEMENT ====================

void moveArmForward() {
  Serial.println("Moving arm forward...");
  armMotor1.run(FORWARD);
  armMotor2.run(FORWARD);
}

void moveArmBackward() {
  Serial.println("Moving arm backward...");
  armMotor1.run(BACKWARD);
  armMotor2.run(BACKWARD);
}

void stopArm() {
  armMotor1.run(RELEASE);
  armMotor2.run(RELEASE);
}

// ==================== GRIPPER MOVEMENT ====================

// Wider Gripper Open
void openGripperWide() {
  Serial.println("Opening gripper wide...");
  gripperMotor.setSpeed(255);  // Max speed to open fully
  gripperMotor.run(FORWARD);
  delay(1000);  // Increased delay for full open
  gripperMotor.run(RELEASE);
}

// Close Gripper Completely
void closeGripper() {
  Serial.println("Closing gripper...");
  gripperMotor.setSpeed(255);  // Ensure tight close
  gripperMotor.run(BACKWARD);
  delay(1000);  // Allow time to fully close
  gripperMotor.run(RELEASE);
}

// ==================== BASE ROTATION ====================

// Rotate base to drop position (slower speed)
void rotateBase() {
  Serial.println("Rotating base...");
  baseMotor.setSpeed(100);  // Slow speed for smooth rotation
  baseMotor.run(FORWARD);
  delay(700);
  baseMotor.run(RELEASE);
}

// Rotate base back to original position
void rotateBaseBack() {
  Serial.println("Rotating base back to original position...");
  baseMotor.setSpeed(100);  // Slow speed for smooth return
  baseMotor.run(BACKWARD);
  delay(700);
  baseMotor.run(RELEASE);
}

// ==================== STOP ALL MOTORS ====================

void stopAllMotors() {
  stopArm();
  gripperMotor.run(RELEASE);
  baseMotor.run(RELEASE);
}
