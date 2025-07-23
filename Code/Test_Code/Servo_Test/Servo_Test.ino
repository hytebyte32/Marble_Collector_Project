#include <ESP32Servo.h>

// === CONFIG ===
const int SERVO_PIN = 4;
const int POT_PIN = 36;  // Make sure this is an ADC-capable pin
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;

// === Objects & State ===
Servo myServo;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Attaching servo...");
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  Serial.println("Servo attached.");
}

void loop() {
  // Read potentiometer (ADC 0–4095)
  int potValue = analogRead(POT_PIN);
  
  // Map pot value to microseconds for the SG90
  int servoUS = map(potValue, 0, 4095, SERVO_MIN_US, SERVO_MAX_US);

  // Clamp for safety
  servoUS = constrain(servoUS, SERVO_MIN_US, SERVO_MAX_US);

  // Move the servo
  myServo.writeMicroseconds(servoUS);

  // Debug output
  Serial.printf("Pot: %4d → Servo: %4d µs\n", potValue, servoUS);

  delay(20);  // Update ~50 Hz
}
