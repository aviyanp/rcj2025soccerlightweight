/*
 * Author: Q Gallant
 * Date: 2023-11-08
 * Version: 1.0
 * Description: This program controls four motors using an Arduino. It sets the direction and speed of each motor, 
 * and alternates between running the motors forward and in reverse every 5 seconds.
 */

// Arrays to hold the pins for motor direction and speed. An array is a collection of items (in this case, pin numbers) stored at contiguous memory locations.
const int motorDirectionPins[4] = {4, 12, 8, 7};  //array for the pins that set the direction
const int motorSpeedPins[4] = {3, 11, 5, 6};  //array for the pins that control the speed

void setup() {
  // Initialize serial communication at 115200 bps:
  Serial.begin(115200);
  Serial.println("[DEBUG] Setup: Initializing motor pins and enabling motor driver.");
  // Set motor direction and speed pins as OUTPUT
  // The 'for' loop is used to repeat a block of code for a certain number of times. In this case, it repeats 4 times, once for each motor.
  for (int i = 0; i < 4; i++) {
    pinMode(motorDirectionPins[i], OUTPUT); // Set the direction pin for motor[i] as OUTPUT
    pinMode(motorSpeedPins[i], OUTPUT); // Set the speed pin for motor[i] as OUTPUT
    Serial.print("[DEBUG] Pin setup: Motor ");
    Serial.print(i);
    Serial.print(" DIR pin ");
    Serial.print(motorDirectionPins[i]);
    Serial.print(", PWM pin ");
    Serial.println(motorSpeedPins[i]);
  }
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  Serial.println("[DEBUG] Motor driver enabled (pin 10 set HIGH)");
}

void loop() {
  Serial.println("[DEBUG] Loop: Running motors forward (clockwise)");
  // Run motors forward
  // Again, we use a 'for' loop to repeat the block of code for each motor.
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorDirectionPins[i], HIGH); // Set direction to forward for motor[i]
    analogWrite(motorSpeedPins[i], 150); // Set speed to 150 (out of 255) for motor[i]
    Serial.print("[DEBUG] Motor ");
    Serial.print(i);
    Serial.print(" DIR=HIGH, PWM=150\n");
  }
  Serial.println("clockwise"); // Print direction to serial monitor
  Serial.println("high"); // Print speed to serial monitor
  delay(5000); // Wait for 5 seconds

  Serial.println("[DEBUG] Loop: Running motors in reverse (counterclockwise)");
  // Run motors in reverse
  // We use another 'for' loop to repeat the block of code for each motor.
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorDirectionPins[i], LOW); // Set direction to reverse for motor[i]
    analogWrite(motorSpeedPins[i], 150); // Set speed to 150 (out of 255) for motor[i]
    Serial.print("[DEBUG] Motor ");
    Serial.print(i);
    Serial.print(" DIR=LOW, PWM=150\n");
  }
  Serial.println("counterclockwise"); // Print direction to serial monitor
  Serial.println("low"); // Print speed to serial monitor
  delay(5000); // Wait for 5 seconds
}
