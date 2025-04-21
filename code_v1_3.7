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

// Variables
int sensorValues[NUM_SENSORS];
int strongestSignalIndex = -1;
int strongestSignalValue = 0;
float initialHeading = 0.0;
float currentHeading = 0.0;

// Thresholds
const int STOP_THRESHOLD = 800;  // Stop threshold for the ball
const int ALIGN_THRESHOLD = 700; // Alignment threshold for the ball

// Function prototypes
void initializeIMU();
float getHeading();
void findStrongestSignal();
void moveTowardsBall();
void alignWithBall();
void pushBallStraight();

void setup() {
    Serial.begin(115200);

    // Initialize IR sensor pins
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
}

void loop() {
    // Step 1: Find the strongest IR signal
    findStrongestSignal();

    // Step 2: If a strong signal is detected, align with the ball
    if (strongestSignalValue >= ALIGN_THRESHOLD) {
        alignWithBall();
        pushBallStraight();
    } else {
        // Step 3: Move toward the ball
        moveTowardsBall();
    }
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
    currentHeading = getHeading();
    while (abs(currentHeading - initialHeading) > 5.0) { // Adjust alignment threshold as needed
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
    stopMotors();
}

void pushBallStraight() {
    int speed = 200;
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
