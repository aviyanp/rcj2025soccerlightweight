/*
 * Author: Q Gallant
 * Date: 2025-04-26
 * Version: 1.0
 * Description: Controls a 3-wheel omniwheel robot with wheels spaced 120 degrees apart.
 * Allows omnidirectional movement by controlling motor speeds and directions.
 */

// Motor 0 (0°)
const int motor0Dir = 4;   // Direction pin
const int motor0Speed = 3; // Speed pin (PWM)

// Motor 1 (120°)
const int motor1Dir = 8;   // Direction pin
const int motor1Speed = 5; // Speed pin (PWM)

// Motor 2 (240°)
const int motor2Dir = 7;   // Direction pin
const int motor2Speed = 6; // Speed pin (PWM)

// Function to set motor speed and direction
void setMotor(int dirPin, int speedPin, float speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward direction
  } else {
    digitalWrite(dirPin, LOW); // Reverse direction
    speed = -speed;            // Convert speed to positive
  }
  analogWrite(speedPin, constrain(speed, 0, 255)); // Write speed (0-255)
}

void setup() {
  // Set all motor pins as outputs
  pinMode(motor0Dir, OUTPUT);
  pinMode(motor0Speed, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
}

void loop() {
  // Example: Move straight forward
  move(0, 100); // Direction: 0° (forward), Speed: 100
  delay(2000);

  // Example: Move straight backward
  move(180, 100); // Direction: 180° (backward), Speed: 100
  delay(2000);

  // Example: Move sideways to the left
  move(90, 100); // Direction: 90° (left), Speed: 100
  delay(2000);

  // Example: Rotate in place clockwise
  rotate(50); // Angular velocity: 50
  delay(2000);

  // Example: Rotate in place counterclockwise
  rotate(-50); // Angular velocity: -50
  delay(2000);

  // Stop moving
  stop();
  delay(2000);
}

// Function to move the robot in a specified direction
void move(float angle, float speed) {
  // Convert angle to radians
  float rad = radians(angle);

  // Calculate motor speeds based on omniwheel kinematics
  float v0 = speed * cos(rad);                // Motor 0 (0°)
  float v1 = speed * cos(rad - radians(120)); // Motor 1 (120°)
  float v2 = speed * cos(rad - radians(240)); // Motor 2 (240°)

  // Set motor speeds
  setMotor(motor0Dir, motor0Speed, v0);
  setMotor(motor1Dir, motor1Speed, v1);
  setMotor(motor2Dir, motor2Speed, v2);
}

// Function to rotate the robot in place
void rotate(float angularVelocity) {
  // All motors contribute to rotation
  setMotor(motor0Dir, motor0Speed, angularVelocity);
  setMotor(motor1Dir, motor1Speed, angularVelocity);
  setMotor(motor2Dir, motor2Speed, angularVelocity);
}

// Function to stop all motors
void stop() {
  setMotor(motor0Dir, motor0Speed, 0);
  setMotor(motor1Dir, motor1Speed, 0);
  setMotor(motor2Dir, motor2Speed, 0);
}