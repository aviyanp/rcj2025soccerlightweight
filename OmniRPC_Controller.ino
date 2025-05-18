/*
 * OmniRPC_Controller.ino
 * Author: GitHub Copilot
 * Date: 2025-05-18
 * 
 * Description: Controls a 4-wheel omnidirectional robot using RPC communication
 * with an OpenMV camera for ball and goal detection.
 */

// Motors pin configuration
const int motorDirectionPins[4] = {4, 12, 8, 7};  // Direction pins for motors 0-3
const int motorSpeedPins[4] = {3, 11, 5, 6};      // Speed pins (PWM) for motors 0-3
const int enablePin = 10;                          // Enable pin for motor driver

// Movement parameters
const int BASE_SPEED = 150;       // Standard movement speed (0-255)
const int SLOW_SPEED = 100;       // Slower speed for precision (0-255)
const int TURN_SPEED = 120;       // Speed for rotation
const float DISTANCE_THRESHOLD = 15.0;  // CM - when to slow down
const float CLOSE_THRESHOLD = 5.0;      // CM - when to consider "arrived"
const float ANGLE_MARGIN = 10.0;        // Degrees - precision for angle alignment

// Communication parameters
const int BUFFER_SIZE = 256;    // Buffer size for RPC messages
char buffer[BUFFER_SIZE];       // Buffer for incoming messages
int bufferIndex = 0;            // Current position in buffer
bool messageComplete = false;   // Whether a complete message has been received

// Object detection data
bool ballDetected = false;         // Whether ball is currently detected
float ballAngle = 0.0;             // Angle to ball in degrees (0-360, 0 is to the right)
float ballDistance = 100.0;        // Distance to ball in cm
float ballConfidence = 0.0;        // Confidence of ball detection (0-100)

bool yellowGoalDetected = false;   // Whether yellow goal is detected
float yellowGoalAngle = 0.0;       // Angle to yellow goal
float yellowGoalDistance = 100.0;  // Distance to yellow goal

bool blueGoalDetected = false;     // Whether blue goal is detected
float blueGoalAngle = 0.0;         // Angle to blue goal
float blueGoalDistance = 100.0;    // Distance to blue goal

// Timing variables
unsigned long lastDetectionTime = 0;      // Last time an object was detected
const unsigned long TIMEOUT_MS = 1000;    // Time without detection before stopping
unsigned long lastDebugTime = 0;          // Time of last debug output
const unsigned long DEBUG_INTERVAL = 500; // Interval for debug output

// Function declarations
void parseRPCMessage(const char* message);
void moveTowardsBall();
void rotateToAngle(float targetAngle);
void moveOmniDirectional(float angle, float speed);
void stopMotors();
void setMotorSpeeds(int m0, int m1, int m2, int m3);

void setup() {
  // Initialize serial communication at 115200 bps
  Serial.begin(115200);

  // Configure motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(motorDirectionPins[i], OUTPUT);
    pinMode(motorSpeedPins[i], OUTPUT);
  }
  
  // Configure and enable motor driver
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  
  // Initialize all motors to stopped
  stopMotors();
  
  Serial.println("Omnidirectional Robot Controller Initialized");
  Serial.println("Waiting for OpenMV RPC data...");
}

void loop() {
  // Check for incoming RPC data from OpenMV
  while (Serial.available() > 0) {
    // Read a character
    char c = Serial.read();
    
    // Process complete messages on newline
    if (c == '\n') {
      // Null-terminate the string
      buffer[bufferIndex] = '\0';
      
      // Process the message
      messageComplete = true;
      
      // Reset buffer for next message
      bufferIndex = 0;
    } 
    // Add character to buffer if there's room
    else if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;
    }
  }
  
  // Process complete messages
  if (messageComplete) {
    parseRPCMessage(buffer);
    messageComplete = false;
  }
  
  // Check for detection timeout
  if (millis() - lastDetectionTime > TIMEOUT_MS) {
    // Stop if we haven't seen the ball recently
    if (ballDetected) {
      ballDetected = false;
      stopMotors();
      Serial.println("Ball detection timed out, stopping");
    }
  }
  
  // Move based on ball position if detected
  if (ballDetected && ballConfidence > 60) {
    moveTowardsBall();
    lastDetectionTime = millis();
  }
  
  // Print debug info periodically
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    if (ballDetected) {
      Serial.print("Ball: Angle=");
      Serial.print(ballAngle);
      Serial.print("° Distance=");
      Serial.print(ballDistance);
      Serial.print("cm Confidence=");
      Serial.println(ballConfidence);
    }
    lastDebugTime = millis();
  }
}

void parseRPCMessage(const char* message) {
  // Look for the ball data section
  char* ballSection = strstr(message, "\"ball\":");
  if (ballSection != NULL) {
    char* foundStr = strstr(ballSection, "\"found\":");
    if (foundStr != NULL && strstr(foundStr, "true") != NULL) {
      // Ball was found
      ballDetected = true;
      
      // Extract angle
      char* angleStr = strstr(ballSection, "\"angle\":");
      if (angleStr != NULL) {
        ballAngle = atof(angleStr + 8); // Skip "angle":
      }
      
      // Extract distance
      char* distStr = strstr(ballSection, "\"distance\":");
      if (distStr != NULL) {
        ballDistance = atof(distStr + 11); // Skip "distance":
      }
      
      // Extract confidence
      char* confStr = strstr(ballSection, "\"confidence\":");
      if (confStr != NULL) {
        ballConfidence = atof(confStr + 13); // Skip "confidence":
      }
      
      // Update last detection time
      lastDetectionTime = millis();
    } else {
      ballDetected = false;
    }
  }
  
  // Parse yellow goal data
  char* yellowSection = strstr(message, "\"yellow_goal\":");
  if (yellowSection != NULL) {
    char* foundStr = strstr(yellowSection, "\"found\":");
    if (foundStr != NULL && strstr(foundStr, "true") != NULL) {
      yellowGoalDetected = true;
      
      // Extract angle
      char* angleStr = strstr(yellowSection, "\"angle\":");
      if (angleStr != NULL) {
        yellowGoalAngle = atof(angleStr + 8);
      }
      
      // Extract distance
      char* distStr = strstr(yellowSection, "\"distance\":");
      if (distStr != NULL) {
        yellowGoalDistance = atof(distStr + 11);
      }
    } else {
      yellowGoalDetected = false;
    }
  }
  
  // Parse blue goal data
  char* blueSection = strstr(message, "\"blue_goal\":");
  if (blueSection != NULL) {
    char* foundStr = strstr(blueSection, "\"found\":");
    if (foundStr != NULL && strstr(foundStr, "true") != NULL) {
      blueGoalDetected = true;
      
      // Extract angle
      char* angleStr = strstr(blueSection, "\"angle\":");
      if (angleStr != NULL) {
        blueGoalAngle = atof(angleStr + 8);
      }
      
      // Extract distance
      char* distStr = strstr(blueSection, "\"distance\":");
      if (distStr != NULL) {
        blueGoalDistance = atof(distStr + 11);
      }
    } else {
      blueGoalDetected = false;
    }
  }
}

void moveTowardsBall() {
  // First check if we need to rotate to face the ball
  // Calculate the shortest way to turn to the target angle
  float angleDiff = ballAngle - 180; // Convert to robot frame (0 is forward, -180 to +180)
  
  // Normalize to -180 to 180 for easier control
  while (angleDiff > 180) angleDiff -= 360;
  while (angleDiff < -180) angleDiff += 360;
  
  // Choose appropriate movement based on ball position
  if (abs(angleDiff) < ANGLE_MARGIN || ballDistance < DISTANCE_THRESHOLD) {
    // Ball is roughly in front or very close, move directly toward it
    int speed = BASE_SPEED;
    
    // Slow down as we get closer
    if (ballDistance < DISTANCE_THRESHOLD) {
      speed = map(ballDistance, CLOSE_THRESHOLD, DISTANCE_THRESHOLD, SLOW_SPEED/2, SLOW_SPEED);
      speed = constrain(speed, SLOW_SPEED/2, SLOW_SPEED);
    }
    
    // Move in the direction of the ball
    moveOmniDirectional(ballAngle, speed);
    
    if (ballDistance <= CLOSE_THRESHOLD) {
      // We're very close to the ball, optionally perform action
      // like pushing the ball toward the goal
      Serial.println("Ball reached!");
    }
  } else {
    // Need to rotate significantly first
    rotateToAngle(ballAngle);
  }
}

// Rotate the robot to face a specific angle
void rotateToAngle(float targetAngle) {
  // Normalize to 0-360
  while (targetAngle < 0) targetAngle += 360;
  while (targetAngle >= 360) targetAngle -= 360;
  
  // Calculate rotation direction (shortest path)
  float currentAngle = 180; // Assume front of robot is 180 degrees in camera coordinates
  float angleDiff = targetAngle - currentAngle;
  
  // Normalize difference to -180 to 180
  while (angleDiff > 180) angleDiff -= 360;
  while (angleDiff < -180) angleDiff += 360;
  
  // Set rotation speed based on angle difference
  int rotationSpeed = map(abs(angleDiff), 0, 180, TURN_SPEED/4, TURN_SPEED);
  rotationSpeed = constrain(rotationSpeed, TURN_SPEED/4, TURN_SPEED);
  
  // Determine rotation direction
  if (angleDiff > 0) {
    // Rotate counterclockwise (all motors counterclockwise)
    for (int i = 0; i < 4; i++) {
      digitalWrite(motorDirectionPins[i], LOW);
      analogWrite(motorSpeedPins[i], rotationSpeed);
    }
  } else {
    // Rotate clockwise (all motors clockwise)
    for (int i = 0; i < 4; i++) {
      digitalWrite(motorDirectionPins[i], HIGH);
      analogWrite(motorSpeedPins[i], rotationSpeed);
    }
  }
}

// Move the robot in any direction using omnidirectional wheels
void moveOmniDirectional(float angle, float speed) {
  // Convert angle from degrees to radians
  float angleRad = angle * PI / 180.0;
  
  // Component vectors for X and Y
  float xComponent = cos(angleRad);
  float yComponent = sin(angleRad);
  
  // Calculate individual motor speeds based on the direction
  // These formulas depend on your specific wheel arrangement and may need adjustment
  int motor0Speed = speed * (-xComponent + yComponent); // Front-left wheel
  int motor1Speed = speed * (-xComponent - yComponent); // Front-right wheel
  int motor2Speed = speed * (xComponent - yComponent);  // Rear-right wheel
  int motor3Speed = speed * (xComponent + yComponent);  // Rear-left wheel
  
  // Apply motor speeds
  setMotorSpeeds(motor0Speed, motor1Speed, motor2Speed, motor3Speed);
}

// Set speeds for all four motors
void setMotorSpeeds(int m0, int m1, int m2, int m3) {
  int motorSpeeds[4] = {m0, m1, m2, m3};
  
  // Apply speeds to each motor
  for (int i = 0; i < 4; i++) {
    int speed = motorSpeeds[i];
    
    // Set direction based on speed sign
    if (speed >= 0) {
      digitalWrite(motorDirectionPins[i], HIGH);
    } else {
      digitalWrite(motorDirectionPins[i], LOW);
      speed = -speed; // Make speed positive for PWM
    }
    
    // Apply PWM (constrain to valid range)
    speed = constrain(speed, 0, 255);
    analogWrite(motorSpeedPins[i], speed);
  }
}

// Stop all motors
void stopMotors() {
  for (int i = 0; i < 4; i++) {
    analogWrite(motorSpeedPins[i], 0);
  }
}