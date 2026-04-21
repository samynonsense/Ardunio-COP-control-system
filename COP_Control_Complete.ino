// Complete COP Control System Arduino Code

#include <Servo.h>

// Define your variables here
int potPin = A0;  // Potentiometer pin
int val;

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(9);  // Attach the servo to pin 9
  Serial.begin(9600);  // Start the serial communication
}

void loop() {
  val = analogRead(potPin);  // Read the potentiometer value
  val = map(val, 0, 1023, 0, 180);  // Map the value to servo angle
  myServo.write(val);  // Set the servo position
  Serial.println(val);  // Print the value for debugging
  delay(15);  // Wait for the servo to reach the position
}