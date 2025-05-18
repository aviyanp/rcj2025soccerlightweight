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
const int ARDUINO_RX_PIN = 10; // RX pin for Arduino (connects to OpenMV TX)
const int ARDUINO_TX_PIN = 9;  // TX pin for Arduino (connects to OpenMV RX)
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
void parseMessage(const char* message); // Renamed from parseRPCMessage
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
  
  // Initialize Serial1 for UART communication with OpenMV
#if defined(ARDUINO_AVR_UNO)
  // On Arduino Uno, Serial1 is not available. Use SoftwareSerial.
  #include <SoftwareSerial.h>
  static SoftwareSerial OpenMVSerial(ARDUINO_RX_PIN, ARDUINO_TX_PIN); // RX, TX
  OpenMVSerial.begin(115200);
  Serial.println("[DEBUG] SoftwareSerial for OpenMV initialized on pins 10 (RX), 9 (TX)");
#else
  Serial1.begin(115200);
  Serial.println("[DEBUG] Serial1 for OpenMV initialized");
#endif
}

void loop() {
  // Check for incoming UART data from OpenMV
#if defined(ARDUINO_AVR_UNO)
  while (OpenMVSerial.available() > 0) {
    // Read a character
    char c = OpenMVSerial.read();
    
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
#else
  while (Serial1.available() > 0) {
    // Read a character
    char c = Serial1.read();
    
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
#endif
  
  // Process complete messages
  if (messageComplete) {
    parseMessage(buffer); // Call renamed function
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

void parseMessage(const char* message) { // Renamed from parseRPCMessage
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
  // Convert ballAngle (0-360, 0 is right from camera) to robot's frame of reference
  // Assuming 0 degrees for the robot is straight ahead.
  // And camera's 90 degrees (straight up from camera view) is robot's 0 degrees.
  float robotAngleToBall = ballAngle - 90; 
  while (robotAngleToBall < 0) robotAngleToBall += 360;
  while (robotAngleToBall >= 360) robotAngleToBall -= 360;

  // Now robotAngleToBall is 0-360, where 0 is straight ahead of the robot.
  // Convert this to -180 to 180 for easier control logic
  float angleDiff = robotAngleToBall;
  if (angleDiff > 180) angleDiff -= 360; // e.g., 270 becomes -90 (turn left)

  // Choose appropriate movement based on ball position
  if (abs(angleDiff) < ANGLE_MARGIN || ballDistance < DISTANCE_THRESHOLD) {
    // Ball is roughly in front or very close, move directly toward it
    int speed = BASE_SPEED;
    
    // Slow down as we get closer
    if (ballDistance < DISTANCE_THRESHOLD) {
      speed = map(ballDistance, CLOSE_THRESHOLD, DISTANCE_THRESHOLD, SLOW_SPEED/2, SLOW_SPEED);
      speed = constrain(speed, SLOW_SPEED/2, SLOW_SPEED);
    }
    
    // Move in the direction of the ball (use robotAngleToBall for omni-directional movement)
    moveOmniDirectional(robotAngleToBall, speed);
    
    if (ballDistance <= CLOSE_THRESHOLD) {
      // We're very close to the ball, optionally perform action
      stopMotors(); // Stop when very close
      Serial.println("Ball reached!");
    }
  } else {
    // Need to rotate significantly first
    // Rotate to make robotAngleToBall (relative to robot front) close to 0
    rotateToAngle(angleDiff); // Pass the -180 to 180 difference
  }
}

// Rotate the robot to face a specific angle difference
void rotateToAngle(float angleDifference) {
  // angleDifference is -180 to 180. Positive means ball is to the right, negative to the left.
  
  int rotationSpeed = map(abs(angleDifference), 0, 180, TURN_SPEED/2, TURN_SPEED);
  rotationSpeed = constrain(rotationSpeed, TURN_SPEED/4, TURN_SPEED);
  
  // Determine rotation direction
  if (angleDifference > 0) { // Ball is to the right, robot needs to turn right (clockwise)
    // Rotate clockwise (motors: M0,M2 forward; M1,M3 backward for typical X omni)
    // This depends heavily on motor setup and omni wheel type. Assuming standard X-drive:
    // To rotate clockwise: all wheels push tangentially clockwise.
    // M0 (FL), M2 (RR) -> HIGH (forward/clockwise spin of wheel)
    // M1 (FR), M3 (RL) -> LOW (backward/clockwise spin of wheel)
    // This is simplified, actual omni rotation might need specific motor directions.
    // For simple rotation, all wheels can spin in the same direction if mounted appropriately.
    // Let's assume all motors spinning one way rotates the chassis.
    digitalWrite(motorDirectionPins[0], HIGH); analogWrite(motorSpeedPins[0], rotationSpeed);
    digitalWrite(motorDirectionPins[1], LOW);  analogWrite(motorSpeedPins[1], rotationSpeed);
    digitalWrite(motorDirectionPins[2], HIGH); analogWrite(motorSpeedPins[2], rotationSpeed);
    digitalWrite(motorDirectionPins[3], LOW);  analogWrite(motorSpeedPins[3], rotationSpeed);
    Serial.println("Rotating Clockwise");

  } else { // Ball is to the left, robot needs to turn left (counter-clockwise)
    // Rotate counterclockwise
    digitalWrite(motorDirectionPins[0], LOW);  analogWrite(motorSpeedPins[0], rotationSpeed);
    digitalWrite(motorDirectionPins[1], HIGH); analogWrite(motorSpeedPins[1], rotationSpeed);
    digitalWrite(motorDirectionPins[2], LOW);  analogWrite(motorSpeedPins[2], rotationSpeed);
    digitalWrite(motorDirectionPins[3], HIGH); analogWrite(motorSpeedPins[3], rotationSpeed);
    Serial.println("Rotating Counter-Clockwise");
  }
}

// Move the robot in any direction using omnidirectional wheels
void moveOmniDirectional(float angleDegrees, float speed) {
  // angleDegrees is relative to the robot's front (0 is straight ahead).
  float angleRad = angleDegrees * PI / 180.0;
  
  // Calculate X and Y components for movement
  // Y is forward/backward, X is left/right
  float moveX = sin(angleRad) * speed; // Strafe component
  float moveY = cos(angleRad) * speed; // Forward/backward component

  // Motor speeds for a 4-wheel omni-drive (X configuration)
  // M0: Front-Left, M1: Front-Right, M2: Rear-Right, M3: Rear-Left
  // These formulas assume motors are numbered 0,1,2,3 starting front-left and going clockwise.
  // And that HIGH on direction pin is forward spin of the wheel.
  // Adjust signs based on your specific motor wiring and mounting.
  int m0_speed = round(moveY + moveX); // Front-Left
  int m1_speed = round(moveY - moveX); // Front-Right
  int m2_speed = round(moveY - moveX); // Rear-Left (for X-drive, should be same as FR for forward, opposite for strafe)
                            // Corrected: M2 (RL) should be similar to M1 for forward, opposite for strafe. 
                            // Let's use a widely accepted X-Drive model:
  // Motor 0 (Front-Left):  +vy +vx +vt
  // Motor 1 (Front-Right): +vy -vx -vt
  // Motor 2 (Rear-Left):   +vy -vx +vt  -- this is if motors are FL, FR, RL, RR
  // Motor 3 (Rear-Right):  +vy +vx -vt
  // Our pins are {4,3 FL}, {12,11 FR}, {8,5 RL}, {7,6 RR}
  // So motorDirectionPins[0] is FL, [1] is FR, [2] is RL, [3] is RR.

  // Assuming no rotation component for now (vt=0)
  int speeds[4];
  speeds[0] = round(moveY + moveX); // Motor 0 (FL)
  speeds[1] = round(moveY - moveX); // Motor 1 (FR)
  speeds[2] = round(moveY - moveX); // Motor 2 (RL) - Error in previous logic, for X drive RL is often moveY - moveX for forward, moveY + moveX for strafe right
  speeds[3] = round(moveY + moveX); // Motor 3 (RR)
  
  setMotorSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);
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