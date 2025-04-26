/*
 * Teensy 4.1 Dual Motor Control with DRV8874 Drivers
 * 
 * Connections:
 * Motor Controller J3:
 *   - PIN 8: Direction control
 *   - PIN 3: PWM speed control
 * 
 * Motor Controller J4:
 *   - PIN 9: Direction control
 *   - PIN 4: PWM speed control
 * 
 * Created: 2025-04-15
 */

// Pin definitions for motor controller J3
const int MOTOR1_DIR_PIN = 8;  // Direction pin for motor 1
const int MOTOR1_PWM_PIN = 3;  // PWM pin for motor 1

// Pin definitions for motor controller J4
const int MOTOR2_DIR_PIN = 9;  // Direction pin for motor 2
const int MOTOR2_PWM_PIN = 4;  // PWM pin for motor 2

// PWM properties
const int PWM_FREQUENCY = 20000; // 20kHz PWM frequency
const int PWM_RESOLUTION = 12;   // 12-bit resolution (0-4095)
const int MAX_PWM = (1 << PWM_RESOLUTION) - 1; // Maximum PWM value

void setup() {
  // Set pin modes
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);

  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for Serial to initialize (timeout after 3 seconds)
  }
  
  // Configure PWM for motor control
  analogWriteFrequency(MOTOR1_PWM_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR2_PWM_PIN, PWM_FREQUENCY);
  analogWriteResolution(PWM_RESOLUTION);
  
  // Initialize motors to stopped state
  stopMotors();
  
  Serial.println("Dual Motor Controller Initialized");
  Serial.println("Commands:");
  Serial.println("  'M1:speed' - Set motor 1 speed (-100 to 100)");
  Serial.println("  'M2:speed' - Set motor 2 speed (-100 to 100)");
  Serial.println("  'STOP' - Stop both motors");
  Serial.println("  'TEST' - Run motor test sequence");
  runMotorTest();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("M1:")) {
      int speed = command.substring(3).toInt();
      setMotor(1, speed);
      Serial.print("Motor 1 set to: ");
      Serial.println(speed);
    }
    else if (command.startsWith("M2:")) {
      int speed = command.substring(3).toInt();
      setMotor(2, speed);
      Serial.print("Motor 2 set to: ");
      Serial.println(speed);
    }
    else if (command.equals("STOP")) {
      stopMotors();
      Serial.println("Motors stopped");
    }
    else if (command.equals("TEST")) {
      runMotorTest();
    }
    else {
      Serial.println("Unknown command");
    }
  }
}

// Set motor speed and direction
// motorNum: 1 or 2
// speed: -100 to 100 (negative for reverse, positive for forward, 0 to stop)
void setMotor(int motorNum, int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -100, 100);
  
  int dirPin, pwmPin;
  
  if (motorNum == 1) {
    dirPin = MOTOR1_DIR_PIN;
    pwmPin = MOTOR1_PWM_PIN;
  } else {
    dirPin = MOTOR2_DIR_PIN;
    pwmPin = MOTOR2_PWM_PIN;
  }

  // Set direction
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward
  } else {
    digitalWrite(dirPin, LOW);  // Reverse
    speed = -speed;             // Convert to positive for PWM calculation
  }
  
  // Calculate PWM value (map 0-100 to 0-MAX_PWM)
  int pwmValue = map(speed, 0, 100, 0, MAX_PWM);
  analogWrite(pwmPin, pwmValue);
}

// Stop both motors
void stopMotors() {
  // Set PWM to 0 for both motors
  analogWrite(MOTOR1_PWM_PIN, 0);
  analogWrite(MOTOR2_PWM_PIN, 0);
}

// Run a test sequence for the motors
void runMotorTest() {
  Serial.println("Running motor test sequence...");
  
  // Test Motor 1
  Serial.println("Testing Motor 1 Forward");
  setMotor(1, 50);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Motor 1 Reverse");
  setMotor(1, -50);
  delay(2000);
  stopMotors();
  delay(500);
  
  // Test Motor 2
  Serial.println("Testing Motor 2 Forward");
  setMotor(2, 50);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Motor 2 Reverse");
  setMotor(2, -50);
  delay(2000);
  stopMotors();
  
  Serial.println("Test sequence complete");
}
