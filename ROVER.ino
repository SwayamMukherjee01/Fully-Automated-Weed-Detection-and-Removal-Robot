ROVER:

#include <AFMotor.h>

// === Define Motors for Rover on the First L293D Shield ===
AF_DCMotor motorFrontLeft(1);   // Rover Front Left Wheel (M1)
AF_DCMotor motorFrontRight(2);  // Rover Front Right Wheel (M2)
AF_DCMotor motorBackLeft(3);    // Rover Back Left Wheel (M3)
AF_DCMotor motorBackRight(4);   // Rover Back Right Wheel (M4)

void setup() {
  Serial.begin(9600);

  // Set speed for Rover Motors
  motorFrontLeft.setSpeed(255);  
  motorFrontRight.setSpeed(255); 
  motorBackLeft.setSpeed(255);   
  motorBackRight.setSpeed(255);  

  stopRover();
}

void loop() {
  // Move Rover Forward
  moveRoverForward();
  delay(3000);  // Move for 3 seconds
  stopRover();
  delay(1000);
}

// ==================== ROVER MOVEMENT ====================

// Adjusted so M4, M3, and M2 now move *forward correctly*
void moveRoverForward() {
  motorFrontLeft.run(FORWARD);   // M1 moves forward (unchanged)
  motorFrontRight.run(BACKWARD); // M2 was reversed, now corrected
  motorBackLeft.run(BACKWARD);   // M3 was reversed, now corrected
  motorBackRight.run(BACKWARD);  // M4 was reversed, now corrected
}

// Function to stop the rover
void stopRover() {
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}
