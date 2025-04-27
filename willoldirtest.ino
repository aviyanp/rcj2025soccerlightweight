// === Motor Pins ===
const int motor0Dir = 4;  // Motor 0 Direction
const int motor0Speed = 3; // Motor 0 Speed (PWM)
const int motor2Dir = 8;  // Motor 2 Direction
const int motor2Speed = 5; // Motor 2 Speed (PWM)
const int motor3Dir = 7;  // Motor 3 Direction
const int motor3Speed = 6; // Motor 3 Speed (PWM)

// === Sensor Pins ===
const int sensorRight = A1;
const int sensorBack = A2;
const int sensorLeft = A3;

// === Constants ===
const int MIN_BALL_THRESHOLD = 100; // Minimum IR value to be considered ball
const int SIMILARITY_THRESHOLD = 100; // Threshold for ignoring small differences
const int MOTOR_SPEED = 150; // Motor speed value for moving

void setup() {
  Serial.begin(115200); // Begin Serial Communication

  // Motor pin setup
  pinMode(motor0Dir, OUTPUT);
  pinMode(motor0Speed, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor3Dir, OUTPUT);
  pinMode(motor3Speed, OUTPUT);

  // Sensor pin setup
  pinMode(sensorRight, INPUT);
  pinMode(sensorBack, INPUT);
  pinMode(sensorLeft, INPUT);
}

// Function to drive motors based on calculated angle
void drive_kiwi(float angle_deg, float power) {
  float angle_rad = radians(angle_deg); // Convert degrees to radians

  // Calculate speeds for each motor
  float motor0 = sin(angle_rad); // Front motor (Motor 0) should go opposite of current direction
  float motor2 = (sin(angle_rad + radians(120))); // Right motor (Motor 2)
  float motor3 = (sin(angle_rad - radians(120))); // Left motor (Motor 3)

  // Normalize motor speeds to be between -1 and 1
  float maxMotor = max(max(abs(motor0), abs(motor2)), abs(motor3));
  if (maxMotor > 1.0) {
    motor0 /= maxMotor;
    motor2 /= maxMotor;
    motor3 /= maxMotor;
  }

  // Apply the power to each motor
  analogWrite(motor0Speed, abs(motor0 * power));
  digitalWrite(-1*motor0Dir, motor0 > 0 ? HIGH : LOW);

  analogWrite(motor2Speed, abs(motor2 * power));
  digitalWrite(-1*motor2Dir, motor2 > 0 ? HIGH : LOW);

  analogWrite(motor3Speed, abs(motor3 * power));
  digitalWrite(-1*motor3Dir, motor3 > 0 ? HIGH : LOW);
}

void loop() {
  // Read sensor values from each direction
  int rightVal = analogRead(sensorRight);
  int backVal = analogRead(sensorBack);
  int leftVal = analogRead(sensorLeft);

  // Print sensor values to serial monitor for debugging
  Serial.print("Right: ");
  Serial.print(rightVal);
  Serial.print(" | Back: ");
  Serial.print(backVal);
  Serial.print(" | Left: ");
  Serial.println(leftVal);

  // Get the strongest sensor value
  int maxVal = max(rightVal, max(backVal, leftVal));

  // If no strong signal is detected, move forward
  if (maxVal < MIN_BALL_THRESHOLD) {
    Serial.println("No strong IR detected: MOVING FORWARD");
    drive_kiwi(0, MOTOR_SPEED); // Move forward (0 degrees)
    delay(50);
    return;
  }

  // Calculate the difference between sensor readings
  int maxDiff = max(abs(rightVal - backVal), max(abs(rightVal - leftVal), abs(backVal - leftVal)));

  // If sensor values are too similar, move forward
  if (maxDiff < SIMILARITY_THRESHOLD) {
    Serial.println("Sensor values similar: MOVING FORWARD");
    drive_kiwi(0, MOTOR_SPEED); // Move forward (0 degrees)
    delay(50);
    return;
  }

  // Calculate a weighted angle based on sensor strengths
  float angleSum = 0;
  float totalStrength = 0;

  angleSum += rightVal * 90;    // Right sensor at 90 degrees
  totalStrength += rightVal;

  angleSum += backVal * 180;    // Back sensor at 180 degrees
  totalStrength += backVal;

  angleSum += leftVal * 270;    // Left sensor at 270 degrees
  totalStrength += leftVal;

  // Calculate the weighted average angle
  float ballAngle = angleSum / totalStrength;

  Serial.print("Moving towards angle: ");
  Serial.println(ballAngle);

  // Move towards the calculated direction
  drive_kiwi(ballAngle, MOTOR_SPEED);

  delay(50); // Small delay to avoid too much output
}
