/*
 * Robot Controller with OpenMV Vision Integration
 * User: RTTrinGH
 * Date: 2025-04-23 14:54:37
 * 
 * This code integrates an OpenMV camera for ball and goal detection
 * with a 4-motor omnidirectional robot platform.
 */

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// IMU setup for LSM303DLHC
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Pin definitions
#define NUM_SENSORS 12
const int IR_PINS[NUM_SENSORS] = {A11, A10, A12, A13, A14, A15, A0, A1, A2, A3, A4, A5};
const int MOTOR1_PWM = 2;
const int MOTOR1_DIR = 7;
const int MOTOR2_PWM = 3;
const int MOTOR2_DIR = 8;
const int MOTOR3_PWM = 4;
const int MOTOR3_DIR = 9;
const int MOTOR4_PWM = 6;
const int MOTOR4_DIR = 11;

// UART buffer for OpenMV communication
const int BUFFER_SIZE = 100;
char serialBuffer[BUFFER_SIZE];
int bufferPos = 0;

// Variables for IR sensors (kept for fallback)
int sensorValues[NUM_SENSORS];
int strongestSignalIndex = -1;
int strongestSignalValue = 0;

// Variables for orientation
float initialHeading = 0.0;
float currentHeading = 0.0;

// Variables for OpenMV vision detection
bool ballDetected = false;
bool blueGoalDetected = false;
bool yellowGoalDetected = false;

float ballX = 0.0;
float ballY = 0.0;
float ballTheta = 0.0;
float ballDistance = 0.0;
int ballConfidence = 0;

float blueGoalDistance = 0.0;
float blueGoalTheta = 0.0;
int blueGoalConfidence = 0;

float yellowGoalDistance = 0.0;
float yellowGoalTheta = 0.0;

// Timeout variables
unsigned long lastBallTime = 0;
unsigned long lastBlueGoalTime = 0;
unsigned long lastYellowGoalTime = 0;
const unsigned long DETECTION_TIMEOUT = 500; // 500ms without detection = object lost

// Thresholds
const int STOP_THRESHOLD = 800;     // Stop threshold for the ball
const int ALIGN_THRESHOLD = 700;    // Alignment threshold for the ball
const float BALL_CLOSE_DISTANCE = 15.0; // cm - when to push the ball
const float ANGLE_TOLERANCE = 0.15; // radians (~8.6 degrees)

// Function prototypes
void initializeIMU();
float getHeading();
void findStrongestSignal();
void moveTowardsBall();
void alignWithBall();
void pushBallStraight();
void stopMotors();
void readOpenMVData();
void parseOpenMVMessage(const char* message);
void checkDetectionTimeouts();
void moveBasedOnVisionData();

void setup() {
    Serial.begin(115200);    // USB Serial for debugging
    Serial1.begin(57600);    // UART for OpenMV communication

    // Initialize IR sensor pins (kept for fallback)
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(IR_PINS[i], INPUT);
    }

    // Initialize motor control pins
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT);
    pinMode(MOTOR4_DIR, OUTPUT);

    // Initialize IMU
    initializeIMU();
    initialHeading = getHeading();

    Serial.println("Robot initialized and ready.");
    Serial.println("RTTrinGH - 2025-04-23 14:54:37");
}

void loop() {
    // Read data from OpenMV camera
    readOpenMVData();
    
    // Check for timeouts (detections gone stale)
    checkDetectionTimeouts();
    
    // If ball is detected by vision system, use that for movement
    if (ballDetected && ballConfidence > 60) {
        moveBasedOnVisionData();
    } 
    // Fall back to IR sensors if vision system doesn't detect the ball
    else {
        // Find the strongest IR signal
        findStrongestSignal();

        // If a strong signal is detected, align with the ball
        if (strongestSignalValue >= ALIGN_THRESHOLD) {
            alignWithBall();
            pushBallStraight();
        } else {
            // Otherwise move toward the ball based on IR sensors
            moveTowardsBall();
        }
    }
}

// Parse incoming data from OpenMV
void readOpenMVData() {
    while (Serial1.available()) {
        char c = Serial1.read();
        
        // Process complete lines
        if (c == '\n') {
            serialBuffer[bufferPos] = '\0'; // Null-terminate the string
            parseOpenMVMessage(serialBuffer);
            bufferPos = 0; // Reset buffer for next message
        } 
        // Add character to buffer if not full
        else if (bufferPos < BUFFER_SIZE - 1) {
            serialBuffer[bufferPos++] = c;
        }
    }
}

// Parse messages from OpenMV
void parseOpenMVMessage(const char* message) {
    char msgType[10];
    
    // Check message type
    if (sscanf(message, "%9[^,]", msgType) == 1) {
        
        // Ball detection message
        if (strcmp(msgType, "ball") == 0) {
            float xVel, yVel;
            if (sscanf(message, "ball,%f,%f,%f,%f,%f,%d", 
                      &ballX, &ballY, &xVel, &yVel, &ballTheta, &ballConfidence) >= 6) {
                
                ballDetected = true;
                ballDistance = sqrt(ballX*ballX + ballY*ballY); // Calculate distance
                lastBallTime = millis(); // Update timestamp
                
                Serial.print("Ball: Dist=");
                Serial.print(ballDistance);
                Serial.print("cm, Angle=");
                Serial.print(ballTheta * 180.0 / PI); // Convert to degrees
                Serial.println("°");
            }
        }
        
        // Blue goal detection
        else if (strcmp(msgType, "blue") == 0) {
            float width, height;
            if (sscanf(message, "blue,%f,%f,%f,%f,%d", 
                      &blueGoalDistance, &blueGoalTheta, &width, &height, &blueGoalConfidence) >= 5) {
                
                blueGoalDetected = true;
                lastBlueGoalTime = millis(); // Update timestamp
                
                Serial.print("Blue Goal: Dist=");
                Serial.print(blueGoalDistance);
                Serial.print("cm, Angle=");
                Serial.print(blueGoalTheta * 180.0 / PI); // Convert to degrees
                Serial.println("°");
            }
        }
        
        // Yellow goal detection
        else if (strcmp(msgType, "yellow") == 0) {
            float width, height;
            if (sscanf(message, "yellow,%f,%f,%f,%f", 
                      &yellowGoalDistance, &yellowGoalTheta, &width, &height) >= 4) {
                
                yellowGoalDetected = true;
                lastYellowGoalTime = millis(); // Update timestamp
                
                Serial.print("Yellow Goal: Dist=");
                Serial.print(yellowGoalDistance);
                Serial.print("cm, Angle=");
                Serial.print(yellowGoalTheta * 180.0 / PI); // Convert to degrees
                Serial.println("°");
            }
        }
        
        // No ball detection
        else if (strcmp(msgType, "no_ball") == 0) {
            // Only mark ball as not detected if we haven't seen it recently
            // This helps with brief detection losses
            if (millis() - lastBallTime > 100) {
                ballDetected = false;
            }
        }
    }
}

// Check if detections have timed out
void checkDetectionTimeouts() {
    unsigned long currentTime = millis();
    
    // Check ball timeout
    if (currentTime - lastBallTime > DETECTION_TIMEOUT) {
        ballDetected = false;
    }
    
    // Check blue goal timeout
    if (currentTime - lastBlueGoalTime > DETECTION_TIMEOUT) {
        blueGoalDetected = false;
    }
    
    // Check yellow goal timeout
    if (currentTime - lastYellowGoalTime > DETECTION_TIMEOUT) {
        yellowGoalDetected = false;
    }
}

// Movement based on OpenMV vision data
void moveBasedOnVisionData() {
    // If ball is very close, push it straight
    if (ballDistance < BALL_CLOSE_DISTANCE && ballConfidence > 70) {
        pushBallStraight();
        return;
    }
    
    // Normalize angle between -π and π for easier logic
    float theta = ballTheta;
    while (theta > PI) theta -= 2 * PI;
    while (theta < -PI) theta += 2 * PI;
    
    // Calculate base speed
    int speed = 200;
    
    // If we're close to the ball, slow down
    if (ballDistance < BALL_CLOSE_DISTANCE * 1.5) {
        speed = 150;
    }
    
    // Calculate approach angle based on which goal is detected
    float targetAngle = theta;
    
    // If yellow goal is detected, factor it into our approach
    if (yellowGoalDetected) {
        // Try to position ball between robot and yellow goal
        // This is a simple algorithm that can be improved
        targetAngle = theta + 0.2 * (yellowGoalTheta - theta);
    }
    
    // Control motors based on angle
    if (fabs(targetAngle) < ANGLE_TOLERANCE) {
        // Ball is straight ahead, move forward
        analogWrite(MOTOR1_PWM, speed);
        analogWrite(MOTOR2_PWM, speed);
        analogWrite(MOTOR3_PWM, speed);
        analogWrite(MOTOR4_PWM, speed);
        digitalWrite(MOTOR1_DIR, HIGH);
        digitalWrite(MOTOR2_DIR, HIGH);
        digitalWrite(MOTOR3_DIR, HIGH);
        digitalWrite(MOTOR4_DIR, HIGH);
    } 
    else if (targetAngle > 0) {
        // Ball is to the left, turn left
        // For omni-drive robot, we can do better than differential steering
        int turnFactor = min(255, int(fabs(targetAngle) * 150));
        
        analogWrite(MOTOR1_PWM, speed - turnFactor/2);
        analogWrite(MOTOR2_PWM, speed - turnFactor/2);
        analogWrite(MOTOR3_PWM, speed);
        analogWrite(MOTOR4_PWM, speed);
        digitalWrite(MOTOR1_DIR, LOW);
        digitalWrite(MOTOR2_DIR, HIGH);
        digitalWrite(MOTOR3_DIR, HIGH);
        digitalWrite(MOTOR4_DIR, LOW);
    } 
    else {
        // Ball is to the right, turn right
        int turnFactor = min(255, int(fabs(targetAngle) * 150));
        
        analogWrite(MOTOR1_PWM, speed);
        analogWrite(MOTOR2_PWM, speed);
        analogWrite(MOTOR3_PWM, speed - turnFactor/2);
        analogWrite(MOTOR4_PWM, speed - turnFactor/2);
        digitalWrite(MOTOR1_DIR, HIGH);
        digitalWrite(MOTOR2_DIR, LOW);
        digitalWrite(MOTOR3_DIR, LOW);
        digitalWrite(MOTOR4_DIR, HIGH);
    }
    
    // Debug output
    Serial.print("Moving based on vision: angle=");
    Serial.print(targetAngle * 180.0 / PI);
    Serial.print("° distance=");
    Serial.println(ballDistance);
}

void initializeIMU() {
    if (!mag.begin()) {
        Serial.println("Failed to initialize LSM303DLHC!");
        while (1);
    }
}

float getHeading() {
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate the heading in degrees
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;

    // Normalize the heading to 0-360 degrees
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}

void findStrongestSignal() {
    strongestSignalValue = 0;
    strongestSignalIndex = -1;

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(IR_PINS[i]);
        if (sensorValues[i] > strongestSignalValue) {
            strongestSignalValue = sensorValues[i];
            strongestSignalIndex = i;
        }
    }
}

void moveTowardsBall() {
    // Check if OpenMV has detected a goal that we can use for orientation
    if (yellowGoalDetected || blueGoalDetected) {
        currentHeading = getHeading();
        // Try to maintain orientation relative to the field while moving
        // This can be expanded based on strategy
    }
    
    int speed = 200;
    if (strongestSignalIndex < NUM_SENSORS / 2) {
        // Move left
        analogWrite(MOTOR1_PWM, speed);
        analogWrite(MOTOR2_PWM, speed);
        digitalWrite(MOTOR1_DIR, LOW);
        digitalWrite(MOTOR2_DIR, HIGH);
        analogWrite(MOTOR3_PWM, speed);
        analogWrite(MOTOR4_PWM, speed);
        digitalWrite(MOTOR3_DIR, HIGH);
        digitalWrite(MOTOR4_DIR, LOW);
    } else {
        // Move right
        analogWrite(MOTOR1_PWM, speed);
        analogWrite(MOTOR2_PWM, speed);
        digitalWrite(MOTOR1_DIR, HIGH);
        digitalWrite(MOTOR2_DIR, LOW);
        analogWrite(MOTOR3_PWM, speed);
        analogWrite(MOTOR4_PWM, speed);
        digitalWrite(MOTOR3_DIR, LOW);
        digitalWrite(MOTOR4_DIR, HIGH);
    }
}

void alignWithBall() {
    // If we have both goal and ball detection, align with both
    if (ballDetected && (yellowGoalDetected || blueGoalDetected)) {
        // Advanced alignment using vision data
        float targetGoalTheta = yellowGoalDetected ? yellowGoalTheta : blueGoalTheta;
        
        // Try to position so the ball is between robot and goal
        float optimalBallTheta = targetGoalTheta + PI; // Opposite direction of goal
        while (optimalBallTheta > PI * 2) optimalBallTheta -= PI * 2;
        
        // Align until ball is in the right direction
        while (abs(ballTheta - optimalBallTheta) > ANGLE_TOLERANCE) {
            // Update OpenMV data
            readOpenMVData();
            
            int turnSpeed = 150;
            if (ballTheta > optimalBallTheta) {
                // Turn left
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, LOW);
                digitalWrite(MOTOR2_DIR, HIGH);
            } else {
                // Turn right
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, HIGH);
                digitalWrite(MOTOR2_DIR, LOW);
            }
        }
    }
    // Fallback to IMU if vision alignment not possible
    else {
        currentHeading = getHeading();
        while (abs(currentHeading - initialHeading) > 5.0) { 
            int turnSpeed = 150;
            if (currentHeading > initialHeading) {
                // Turn left
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, LOW);
                digitalWrite(MOTOR2_DIR, HIGH);
            } else {
                // Turn right
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, HIGH);
                digitalWrite(MOTOR2_DIR, LOW);
            }
            currentHeading = getHeading();
        }
    }
    stopMotors();
}

void pushBallStraight() {
    // If yellow goal is detected, aim toward it
    if (yellowGoalDetected) {
        // Adjust robot direction to push toward yellow goal
        while (abs(yellowGoalTheta) > ANGLE_TOLERANCE) {
            // Update OpenMV data
            readOpenMVData();
            
            int turnSpeed = 150;
            if (yellowGoalTheta > 0) {
                // Turn left
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, LOW);
                digitalWrite(MOTOR2_DIR, HIGH);
            } else {
                // Turn right
                analogWrite(MOTOR1_PWM, turnSpeed);
                analogWrite(MOTOR2_PWM, turnSpeed);
                digitalWrite(MOTOR1_DIR, HIGH);
                digitalWrite(MOTOR2_DIR, LOW);
            }
        }
    }

    // Push the ball straight ahead with full power
    int speed = 230; // Higher speed for pushing
    analogWrite(MOTOR1_PWM, speed);
    analogWrite(MOTOR2_PWM, speed);
    analogWrite(MOTOR3_PWM, speed);
    analogWrite(MOTOR4_PWM, speed);
    digitalWrite(MOTOR1_DIR, HIGH);
    digitalWrite(MOTOR2_DIR, HIGH);
    digitalWrite(MOTOR3_DIR, HIGH);
    digitalWrite(MOTOR4_DIR, HIGH);

    delay(1500); // Adjust delay to control how far the robot pushes forward
    stopMotors();
}

void stopMotors() {
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);
    analogWrite(MOTOR3_PWM, 0);
    analogWrite(MOTOR4_PWM, 0);
}
