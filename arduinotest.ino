// === Motor Pins ===
const int motor0Dir = 4; // Direction pin
const int motor0Speed = 3; // Speed pin (PWM)
const int motor2Dir = 8; // Direction pin
const int motor2Speed = 5; // Speed pin (PWM)
const int motor3Dir = 7; // Direction pin
const int motor3Speed = 6; // Speed pin (PWM)

// === Sensor Pins ===
const int sensorRight = A1;   // Right
const int sensorBack = A2;    // Back
const int sensorLeft = A3;    // Left

// === Constants ===
const int MIN_BALL_THRESHOLD = 100; // minimum sensor value to be considered seeing the ball
const int SIMILARITY_THRESHOLD = 200; // how close sensor values must be to be considered "similar"

void setup() {
  Serial.begin(115200);

  pinMode(motor0Dir, OUTPUT);
  pinMode(motor0Speed, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor3Dir, OUTPUT);
  pinMode(motor3Speed, OUTPUT);

  pinMode(sensorRight, INPUT);
  pinMode(sensorBack, INPUT);
  pinMode(sensorLeft, INPUT);
}

// Simple motor drive function
void drive_motor(int motor, int speed) {
  if (motor == 0) {
    digitalWrite(motor0Dir, HIGH);
    analogWrite(motor0Speed, speed);
  }
  else if (motor == 2) {
    digitalWrite(motor2Dir, LOW);
    analogWrite(motor2Speed, speed);
  }
  else if (motor == 3) {
    digitalWrite(motor3Dir, HIGH);
    analogWrite(motor3Speed, speed);
  }
}

void move_forward() {
  // Move forward by driving all motors
  drive_motor(0, 150);
  drive_motor(2, 150);
  drive_motor(3, 150);
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
    Serial.println("No strong IR detected: moving FORWARD");
    move_forward();
    delay(50);
    return;
  }

  // Check if sensor values are close together
  int maxDiff = max(abs(rightVal - backVal), max(abs(rightVal - leftVal), abs(backVal - leftVal)));

  if (maxDiff < SIMILARITY_THRESHOLD) {
    Serial.println("Sensor values are similar: moving FORWARD");
    move_forward();
    delay(50);
    return;
  }

  // Otherwise, move towards the strongest direction
  if (maxVal == rightVal) {
    Serial.println("Ball detected RIGHT: turning right");
    drive_motor(2, 150);
    drive_motor(3, 0);
    drive_motor(0, 0);
  }
  else if (maxVal == backVal) {
    Serial.println("Ball detected BACK: turning back");
    drive_motor(0, 150);
    drive_motor(2, 0);
    drive_motor(3, 0);
  }
  else if (maxVal == leftVal) {
    Serial.println("Ball detected LEFT: turning left");
    drive_motor(3, 150);
    drive_motor(2, 0);
    drive_motor(0, 0);
  }

  delay(50);
}
