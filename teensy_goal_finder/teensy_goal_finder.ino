// Define the Teensy pins connected to the OpenMV output pins
// Adjust these pin numbers based on your actual wiring
const int LEFT_PIN = 2;    // Connect to OpenMV P0
const int CENTER_PIN = 3;  // Connect to OpenMV P1
const int RIGHT_PIN = 4;   // Connect to OpenMV P2

// Variable to store the detected goal direction
// Using String for easy readability, can be changed to enum or int for performance
String goalDir = "none";

void setup() {
  // Initialize Serial communication for debugging/output
  Serial.begin(9600);
  // Wait for Serial port to connect (needed for Native USB like Teensy)
  while (!Serial && millis() < 5000) {
    // Wait up to 5 seconds for serial connection
  }
  Serial.println("Teensy Goal Direction Reader Initialized.");
  Serial.println("Reading pins:");
  Serial.print("Left: "); Serial.println(LEFT_PIN);
  Serial.print("Center: "); Serial.println(CENTER_PIN);
  Serial.print("Right: "); Serial.println(RIGHT_PIN);

  // Set the defined pins as inputs
  // Standard INPUT assumes the OpenMV actively drives the pins LOW when not HIGH.
  // If you encounter floating inputs, you might consider INPUT_PULLDOWN,
  // but only if the OpenMV output goes high-impedance instead of LOW.
  pinMode(LEFT_PIN, INPUT);
  pinMode(CENTER_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);

  goalDir = "none"; // Initialize direction
}

void loop() {
  // Read the digital state of each input pin
  int leftState = digitalRead(LEFT_PIN);
  int centerState = digitalRead(CENTER_PIN);
  int rightState = digitalRead(RIGHT_PIN);

  // Determine the goal direction based on which pin is HIGH
  // Assumes only one pin will be HIGH at a time, as per the OpenMV logic.
  if (leftState == HIGH) {
    goalDir = "left";
  } else if (centerState == HIGH) {
    goalDir = "center";
  } else if (rightState == HIGH) {
    goalDir = "right";
  } else {
    // All pins are LOW
    goalDir = "none";
  }

  // --- Optional: Print the detected direction to the Serial Monitor ---
  // You can comment this out if you don't need constant serial output
  static String lastPrintedDir = "";
  if (goalDir != lastPrintedDir) { // Only print when the direction changes
     Serial.print("Goal Direction: ");
     Serial.println(goalDir);
     lastPrintedDir = goalDir;
  }
  // --- End Optional Print ---


  // You can now use the 'goalDir' variable elsewhere in your Teensy code
  // for decision-making, motor control, etc.
  // Example:
  // if (goalDir == "left") {
  //   turnRobotLeft();
  // } else if (goalDir == "center") {
  //   moveRobotForward();
  // } // etc...

  // Add a small delay to prevent the loop from running too fast,
  // though reading digital pins is very quick. Adjust as needed.
  delay(10);
}
