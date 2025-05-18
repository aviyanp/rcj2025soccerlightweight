#include <SoftwareSerial.h>

// —— Motor pins ——  
const int motorDirectionPins[4] = {4, 12, 8, 7};
const int motorSpeedPins[4]     = {3, 11, 5, 6};

// —— SoftwareSerial to OpenMV ——  
// Rx = D10, Tx = D9 (we only read)
SoftwareSerial openmv(10, 9);

// —— State machine ——  
enum State { SEARCH_BALL, APPROACH_BALL, SEARCH_GOAL, APPROACH_GOAL, DONE };
State state = SEARCH_BALL;

// —— Pixel‐area thresholds ——  
const int BALL_POSSESSION_AREA = 5000;   // tune for “close enough” to the ball
const int GOAL_FAST_AREA       = 15000;  // ~20 cm from goal
const int GOAL_STOP_AREA       = 40000;  // ~8 cm from goal

// —— Speeds (gentler) ——  
const int TURN_SPEED     =  50;
const int APPROACH_SPEED =  30;
const int FAST_SPEED     =  80;
const float ANGLE_TOLERANCE = 10.0;  // degrees

// —— Runtime vars ——  
float ballAngle = 0; int ballArea = 0;
float goalAngle = 0; int goalArea = 0;

void setup() {
  Serial.begin(115200);
  openmv.begin(9600);
  Serial.println("▶️ State machine starting…");

  for (int i = 0; i < 4; i++) {
    pinMode(motorDirectionPins[i], OUTPUT);
    pinMode(motorSpeedPins[i], OUTPUT);
  }
  stopAll();
}

void loop() {
  // 1) Read BALL / GOAL messages
  if (openmv.available()) {
    String line = openmv.readStringUntil('\n');
    line.trim();
    Serial.print("📥 "); Serial.println(line);

    if (line.startsWith("BALL")) {
      parseData(line, ballAngle, ballArea);
      Serial.print("  🔴 BALL: angle="); Serial.print(ballAngle,0);
      Serial.print("°, area="); Serial.println(ballArea);
    }
    else if (line.startsWith("GOAL")) {
      parseData(line, goalAngle, goalArea);
      Serial.print("  🔵 GOAL: angle="); Serial.print(goalAngle,0);
      Serial.print("°, area="); Serial.println(goalArea);
    }
  }

  // 2) State transitions
  switch (state) {
    case SEARCH_BALL:
      if (ballArea > 0) {
        Serial.println("→ Found ball: APPROACH_BALL");
        state = APPROACH_BALL;
      } else {
        turnRight(TURN_SPEED);
      }
      break;

    case APPROACH_BALL:
      if (ballArea >= BALL_POSSESSION_AREA) {
        Serial.println("→ Ball secured: SEARCH_GOAL");
        state = SEARCH_GOAL;
      } else {
        driveToward(ballAngle, APPROACH_SPEED);
      }
      break;

    case SEARCH_GOAL:
      if (goalArea > 0) {
        Serial.println("→ Goal spotted: APPROACH_GOAL");
        state = APPROACH_GOAL;
      } else {
        turnRight(TURN_SPEED);
      }
      break;

    case APPROACH_GOAL:
      if      (goalArea >= GOAL_STOP_AREA) { stopAll(); state = DONE;  Serial.println("→ Stopped at goal"); }
      else if (goalArea >= GOAL_FAST_AREA) { driveToward(goalAngle, FAST_SPEED); }
      else                                  { driveToward(goalAngle, APPROACH_SPEED); }
      break;

    case DONE:
      stopAll();
      break;
  }

  delay(100);
}

// — Helpers —

void parseData(const String &s, float &angle, int &area) {
  int c1 = s.indexOf(','), c2 = s.indexOf(',', c1+1);
  angle = s.substring(c1+1, c2).toFloat();
  area  = s.substring(c2+1).toInt();
}

void driveToward(float angle, int sp) {
  if (angle > 180) angle -= 360;
  if (fabs(angle) > ANGLE_TOLERANCE) {
    if (angle > 0) turnRight(sp);
    else           turnLeft(sp);
  } else {
    setSide(true, sp);
  }
}

void turnRight(int sp) {
  setSide(true,  sp);
  setSide(false, sp);
}
void turnLeft(int sp) {
  setSide(false, sp);
  setSide(true,  sp);
}

void setMotor(int idx, bool forward, int sp) {
  bool dir = forward;
  if (idx == 1 || idx == 2) dir = !forward;
  digitalWrite(motorDirectionPins[idx], dir);
  analogWrite (motorSpeedPins[idx], sp);
}

void setSide(bool forward, int sp) {
  // left = motors 0,1; right = motors 2,3
  setMotor(0, forward, sp);
  setMotor(1, forward, sp);
  setMotor(2, forward, sp);
  setMotor(3, forward, sp);
}

void stopAll() {
  for (int i = 0; i < 4; i++) analogWrite(motorSpeedPins[i], 0);
}
