/*
 * X-Drive Robot with IR Ball Tracking
 * 
 * This program controls a robot with X-drive configuration to:
 * - Move horizontally back and forth
 * - Track an IR ball using 3 sensors (0°, -45°, 45°)
 * - Rotate towards the strongest IR signal
 * 
 * Hardware requirements:
 * - Teensy board
 * - 4 motors in X-drive configuration
 * - 3 IR sensors positioned at 0°, -45°, and 45°
 */

// Motor pins (adjust according to your setup)
const int motorFrontLeft = 2;    // Front left motor
const int motorFrontRight = 3;   // Front right motor
const int motorBackLeft = 4;     // Back left motor
const int motorBackRight = 5;    // Back right motor

// PWM enable pins for motors
const int enableFrontLeft = 6;
const int enableFrontRight = 7;
const int enableBackLeft = 8;
const int enableBackRight = 9;

// IR sensor pins
const int irForward = A0;    // Forward IR sensor (0°)
const int irLeft = A1;       // Left IR sensor (45°)
const int irRight = A2;      // Right IR sensor (-45°)

// Constants
const int motorSpeed = 150;      // Default motor speed (0-255)
const int rotationSpeed = 100;   // Speed when rotating
const int irThreshold = 50;      // Minimum IR reading difference to trigger rotation
const int horizontalTime = 2000; // Time in ms to move in one direction
unsigned long lastDirectionChange = 0;
bool movingRight = true;         // Start moving right

void setup() {
  // Initialize motor pins as outputs
  pinMode(motorFrontLeft, OUTPUT);
  pinMode(motorFrontRight, OUTPUT);
  pinMode(motorBackLeft, OUTPUT);
  pinMode(motorBackRight, OUTPUT);
  
  pinMode(enableFrontLeft, OUTPUT);
  pinMode(enableFrontRight, OUTPUT);
  pinMode(enableBackLeft, OUTPUT);
  pinMode(enableBackRight, OUTPUT);
  
  // Initialize IR sensor pins as inputs
  pinMode(irForward, INPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  
  // Start serial for debugging
  Serial.begin(9600);
  Serial.println("X-Drive Robot with IR Tracking initialized");
  
  // Set initial motor speeds
  analogWrite(enableFrontLeft, motorSpeed);
  analogWrite(enableFrontRight, motorSpeed);
  analogWrite(enableBackLeft, motorSpeed);
  analogWrite(enableBackRight, motorSpeed);
}

void loop() {
  // Read IR sensor values
  int forwardValue = analogRead(irForward);
  int leftValue = analogRead(irLeft);
  int rightValue = analogRead(irRight);
  
  // Print IR values for debugging
  Serial.print("Forward: ");
  Serial.print(forwardValue);
  Serial.print(" | Left: ");
  Serial.print(leftValue);
  Serial.print(" | Right: ");
  Serial.println(rightValue);
  
  // Find strongest IR signal and rotate if needed
  int maxReading = max(max(forwardValue, leftValue), rightValue);
  
  if (maxReading > 0) { // Only track if we detect something
    if (maxReading == forwardValue && 
       (forwardValue > leftValue + irThreshold || forwardValue > rightValue + irThreshold)) {
      // Already facing the IR source, don't rotate
      stopRotation();
    } 
    else if (maxReading == leftValue && leftValue > forwardValue + irThreshold) {
      // Rotate left towards the ball
      rotateLeft();
      delay(50); // Small delay to allow rotation
      stopRotation();
    } 
    else if (maxReading == rightValue && rightValue > forwardValue + irThreshold) {
      // Rotate right towards the ball
      rotateRight();
      delay(50); // Small delay to allow rotation
      stopRotation();
    }
  }
  
  // Continue horizontal movement (back and forth)
  unsigned long currentTime = millis();
  
  // Change direction periodically
  if (currentTime - lastDirectionChange > horizontalTime) {
    movingRight = !movingRight;
    lastDirectionChange = currentTime;
  }
  
  // Move horizontally in the current direction
  if (movingRight) {
    moveRight();
  } else {
    moveLeft();
  }
  
  delay(10); // Short delay for stability
}

// Move the robot to the right (relative to robot's perspective)
void moveRight() {
  // For X-drive, all motors contribute to sideways movement
  // Each motor's direction depends on its position in X configuration
  digitalWrite(motorFrontLeft, HIGH);
  digitalWrite(motorFrontRight, LOW);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorBackRight, HIGH);
}

// Move the robot to the left (relative to robot's perspective)
void moveLeft() {
  digitalWrite(motorFrontLeft, LOW);
  digitalWrite(motorFrontRight, HIGH);
  digitalWrite(motorBackLeft, HIGH);
  digitalWrite(motorBackRight, LOW);
}

// Rotate the robot clockwise
void rotateRight() {
  // Set rotation speed
  analogWrite(enableFrontLeft, rotationSpeed);
  analogWrite(enableFrontRight, rotationSpeed);
  analogWrite(enableBackLeft, rotationSpeed);
  analogWrite(enableBackRight, rotationSpeed);
  
  // All motors turn the same direction for rotation
  digitalWrite(motorFrontLeft, HIGH);
  digitalWrite(motorFrontRight, LOW);
  digitalWrite(motorBackLeft, HIGH);
  digitalWrite(motorBackRight, LOW);
}

// Rotate the robot counter-clockwise
void rotateLeft() {
  // Set rotation speed
  analogWrite(enableFrontLeft, rotationSpeed);
  analogWrite(enableFrontRight, rotationSpeed);
  analogWrite(enableBackLeft, rotationSpeed);
  analogWrite(enableBackRight, rotationSpeed);
  
  // All motors turn the same direction for rotation
  digitalWrite(motorFrontLeft, LOW);
  digitalWrite(motorFrontRight, HIGH);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorBackRight, HIGH);
}

// Stop rotation and reset to movement speed
void stopRotation() {
  // Reset to default motor speed
  analogWrite(enableFrontLeft, motorSpeed);
  analogWrite(enableFrontRight, motorSpeed);
  analogWrite(enableBackLeft, motorSpeed);
  analogWrite(enableBackRight, motorSpeed);
}
