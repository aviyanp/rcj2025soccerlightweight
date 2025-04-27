// === Motor Pins ===
const int motor0Dir = 6; // Direction pin for front motor
const int motor0Speed = 3; // Speed pin (PWM) for front motor
const int motor1Dir = 2; // Direction pin for right motor
const int motor1Speed = 5; // Speed pin (PWM) for right motor
const int motor2Dir = 7; // Direction pin for left motor
const int motor2Speed = 6; // Speed pin (PWM) for left motor

// === Sensor Pins ===
const int sensorRight = A1;   // Right IR sensor
const int sensorBack = A2;    // Back IR sensor
const int sensorLeft = A3;    // Left IR sensor

// === Constants ===
const int MIN_BALL_THRESHOLD = 100; // minimum sensor value to be considered seeing the ball
const int SIMILARITY_THRESHOLD = 200; // how close sensor values must be to be considered "similar"
const float L = 4.0; // Distance from center to wheel (in inches)

// === Globals ===
bool signalLost = false; // Tracks whether the IR signal is lost

void setup() {
  Serial.begin(115200);

  pinMode(motor0Dir, OUTPUT);
  pinMode(motor0Speed, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2Speed, OUTPUT);

  pinMode(sensorRight, INPUT);
  pinMode(sensorBack, INPUT);
  pinMode(sensorLeft, INPUT);
}

// Simple motor control function
void drive_motor(int motor, int speed, int direction) {
  int dirPin, speedPin;

  if (motor == 0) { dirPin = motor0Dir; speedPin = motor0Speed; }
  else if (motor == 1) { dirPin = motor1Dir; speedPin = motor1Speed; }
  else if (motor == 2) { dirPin = motor2Dir; speedPin = motor2Speed; }

  digitalWrite(dirPin, direction);
  analogWrite(speedPin, speed);
}

void stop_all_motors() {
  drive_motor(0, 0, LOW);
  drive_motor(1, 0, LOW);
  drive_motor(2, 0, LOW);
}

// Vector-based movement function
void move_vector(float x, float y, float rotation) {
  // Calculating motor speeds using the provided equations
  const float sqrt3o2 = sqrt(3) / 2.0;
  
  float M0 = -y + rotation; // Motor 0 (Front)
  float M1 = 0.5 * y + sqrt3o2 * x + rotation; // Motor 1 (Right)
  float M2 = 0.5 * y - sqrt3o2 * x + rotation; // Motor 2 (Left)

  // Setting directions
  int dir0 = M0 < 0 ? LOW : HIGH;
  int dir1 = M1 < 0 ? LOW : HIGH;
  int dir2 = M2 < 0 ? LOW : HIGH;

  // Setting speeds
  M0 = map(abs(M0), 0, 100, 0, 255);
  M1 = map(abs(M1), 0, 100, 0, 255);
  M2 = map(abs(M2), 0, 100, 0, 255);

  drive_motor(0, M0, dir0);
  drive_motor(1, M1, dir1);
  drive_motor(2, M2, dir2);
}

void loop() {
  int rightVal = analogRead(sensorRight);
  int backVal = analogRead(sensorBack);
  int leftVal = analogRead(sensorLeft);

  Serial.print("Right: ");
  Serial.print(rightVal);
  Serial.print(" | Back: ");
  Serial.print(backVal);
  Serial.print(" | Left: ");
  Serial.println(leftVal);

  int maxVal = max(rightVal, max(backVal, leftVal));

  // If no strong ball detected
  if (maxVal < MIN_BALL_THRESHOLD) {
    Serial.println("No strong IR detected: executing STRAFING maneuver");
    signalLost = true;

    // Strafe diagonally forward and slightly opposite direction
    move_vector(-30, 50, 0); // Adjust X, Y, and rotation values as necessary
    delay(1000); // Execute the maneuver for 1 second
    return;
  }

  signalLost = false;

  // Check if sensor values are close together
  int maxDiff = max(abs(rightVal - backVal), max(abs(rightVal - leftVal), abs(backVal - leftVal)));

  if (maxDiff < SIMILARITY_THRESHOLD) {
    Serial.println("Sensor values are similar: moving FORWARD");
    move_vector(0, 50, 0); // Move forward
    delay(50);
    return;
  }

  // Otherwise, move towards the strongest direction
  if (maxVal == rightVal) {
    Serial.println("Ball detected RIGHT: moving right");
    move_vector(50, 0, 0);
  }
  else if (maxVal == backVal) {
    Serial.println("Ball detected BACK: moving backward");
    move_vector(0, -50, 0);
  }
  else if (maxVal == leftVal) {
    Serial.println("Ball detected LEFT: moving left");
    move_vector(-50, 0, 0);
  }

  delay(50);
}
