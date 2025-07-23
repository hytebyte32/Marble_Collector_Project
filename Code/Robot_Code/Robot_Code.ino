#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <algorithm>
#include <ESP32Servo.h>

// Button structure
struct Button {
  const int pin;
  volatile uint32_t numberPresses;
  uint32_t nextPressTime;
  volatile bool pressed;
};

// Encoder structure
struct Encoder {
  const int PinA;
  const int PinB;
  volatile long pos;
};

// servo status structr=ure
struct servoStatus {
  uint32_t startTime;
  uint16_t startPos;
  uint16_t moveDuration;
  uint16_t currentPos;
  uint16_t targetPos;
};

// Function declarations
void setMotor(int dir, int pwm, int in1, int in2);
void setServo(int index);
void setServoStatus(int index, uint16_t moveDuration, uint16_t targetPos);
void detectColor();
void normalizeColor();
void doHeartbeat();
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Constants
const int cHeartbeatInterval = 75;
const int cSmartLED = 23;
const int cSmartLEDCount = 1;
const long cDebounceDelay = 170;
const int cNumMotors = 2;
const int cPWMRes = 8;
const int cMinPWM = 150;
const int cMaxPWM = pow(2, cPWMRes) - 1;
const int cArmMax = 950;
const int cArmMin = 2400;
const int cArmStandby = 1550;
const int cGateMin = 2050;
const int cGateMax = 600;
const int cPWMFreq = 20000;
const int cCountsRev = 1096;
const int cPotPin = 36;
const int cSDA = 18;
const int cSCL = 19;
const int cTCSLED = 14;
const int cIN1Pin[] = {26, 16};
const int cIN2Pin[] = {27, 17};
const uint32_t servoWaitDuration = 2000000;

int cMotorAdjustment[] = {0, 0};

// Variables
uint32_t lastHeartbeat = 0;
uint32_t lastServoMoveTime = 0;
uint32_t curMillis = 0;
uint32_t robotModeIndex = 0;
uint32_t subModeIndex[] = {0, 0, 0, 0, 0};
uint32_t lastModeIndex = 0;
uint32_t lastTime = 0;
uint32_t absEncoderPos = 0;
uint32_t totalEncoderPos = 0;
uint16_t baselineColors[4][11] = {0};
float baselineColor[4] = {0};
float baselineGreen[4] = {0.297, 0.405, 0.297, 28};
uint16_t r, g, b, c;
float rNorm, gNorm, bNorm;
uint16_t encoderOffset = 0;
uint8_t driveSpeed = 0;
uint8_t startButton = 0;
bool detectedGreen = false;
uint32_t timerCount = 0;
bool detectedColor = false;
bool waitingForServo = false;
uint32_t waitStartTime = 0;
uint32_t twoMinuteTimer = 0;

Button modeButton = {0, 0, 0, false};
Encoder encoder[] = {
  {35, 32, 0},
  {33, 25, 0}
};

Servo servo[2];
servoStatus servoStatus[2];


Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = {
  0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135,
  150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0
};

// TCS34725 flag
bool tcsFlag = 0;

uint32_t modeIndicator[6] = {
  SmartLEDs.Color(255, 0, 0),
  SmartLEDs.Color(0, 255, 0),
  SmartLEDs.Color(0, 0, 255),
  SmartLEDs.Color(255, 255, 0),
  SmartLEDs.Color(0, 255, 255),
  SmartLEDs.Color(255, 0, 255),
};

void setup() {
  Serial.begin(115200);

  // Initialize smart LED
  SmartLEDs.begin();
  SmartLEDs.clear();
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));
  SmartLEDs.setBrightness(0);
  SmartLEDs.show();

  // Initialize encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);
    pinMode(encoder[k].PinA, INPUT);
    pinMode(encoder[k].PinB, INPUT);
    attachInterruptArg(encoder[k].PinA, encoderISR, &encoder[k], RISING);
  }

  // Initialize push button
  pinMode(modeButton.pin, INPUT_PULLUP);
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING);

  // Initialize potentiometer 
  pinMode(cPotPin, INPUT);

  // Intialize color sensor
  Wire.setPins(cSDA, cSCL);
  pinMode(cTCSLED, OUTPUT);
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  // Enables color sensor LED
  digitalWrite(cTCSLED, true);
  delay(200);

  // Collects 10 samples from color sensor
  for (int i = 0; i < 10; i++) {
    if (tcsFlag) {
      tcs.getRawData(&r, &g, &b, &c);
      baselineColors[0][i] = r;
      baselineColors[1][i] = g;
      baselineColors[2][i] = b;
      baselineColors[3][i] = c;
    }
  }

  // Finds median value for each channel
  for (int i = 0; i < 4; i++) {
    uint16_t temp[10];
    memcpy(temp, baselineColors[i], sizeof(temp));
    std::sort(temp, temp + 10);
    baselineColor[i] = (float)temp[5];
  }

  float totalRGB = baselineColor[0] + baselineColor[1] + baselineColor[2];
  baselineColor[0] /= totalRGB;
  baselineColor[1] /= totalRGB;
  baselineColor[2] /= totalRGB;

  // Shows baseline values in serial monitor
  Serial.println("Baseline Color Sensor Readings (10 samples):");
  Serial.println("Index\tR\tG\tB\tC");
  for (int i = 0; i < 10; i++) {
    Serial.print(i); Serial.print("\t");
    Serial.print(baselineColors[0][i]); Serial.print("\t");
    Serial.print(baselineColors[1][i]); Serial.print("\t");
    Serial.print(baselineColors[2][i]); Serial.print("\t");
    Serial.println(baselineColors[3][i]);
  }
  Serial.printf("Normalized R %.3f, G: %.3f, B: %.3f, C: %.3f\n", baselineColor[0], baselineColor[1], baselineColor[2], baselineColor[3]);

  // Initialize servos
  servo[0].attach(4);  // Arm
  servo[1].attach(13); // Gate
  servoStatus[0] = {0, 0, 0, cArmMax, cArmMax};
  servoStatus[1] = {0, 0, 0, cGateMax, cGateMax};
  servo[0].writeMicroseconds(servoStatus[0].currentPos);
  delay(2000);
  servo[1].writeMicroseconds(servoStatus[1].currentPos);
  delay(2000);

  twoMinuteTimer = millis();
}


void loop() {

  doHeartbeat();

  long pos[] = {0, 0};
  int pot = 0;

  noInterrupts();
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;
  }
  interrupts();

  // Calculate motor speed
  pot = analogRead(cPotPin);
  driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
  absEncoderPos = (abs(pos[0]) + abs(pos[1])) / 2;

  // Moves servo

  // Stops robot for 2s if servo is moving
  static bool printedWaitMsg = false;
  if (waitingForServo) {
    setServo(0);
    setServo(1);
    if (millis() - waitStartTime >= 2000) {
      waitingForServo = false;
      printedWaitMsg = false;

    } else {
      setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
      setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
      return;
    }
  }


  // Start button
  if (modeButton.pressed) {
    startButton ^= 1;
    modeButton.pressed = false;
    if (startButton) {
      robotModeIndex = 0;
      memset(subModeIndex, 0, sizeof(subModeIndex));
      totalEncoderPos = 0;
      encoder[0].pos = 0;
      encoder[1].pos = 0;
    }
  }

  switch (startButton) {
    // Calibration Loop
    case 0:
      break;

    // Main Collection Loop
    case 1:
        switch (robotModeIndex) {

        // Search for gem
        case 0:
          switch (subModeIndex[0]) {
            
            // Open gate
            case 0:
              Serial.println("Seach - Opening Gate");
              setServoStatus(1, 0, cGateMax);

              subModeIndex[0] = 1;
              break;
            // Drive forward
            case 1:
              Serial.println("Search - Driving Forward");
              setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
              setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

              subModeIndex[0] = 2;

              break;

            // Detect for gem
            case 2:
              Serial.println("Search - Detecting for Gem");
              detectColor();
              if (detectedColor) {
                
                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[0] = 3;
              }
              break;

            // Move forward to get gem into collection chamber
            case 3:
              Serial.println("Search - Isolating Gem");
              encoderOffset = 175;
              if (absEncoderPos <= encoderOffset) {
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[0] = 4;
              }
              break;

            // Close gate to isolate gem
            case 4:
              Serial.println("Search - Closing Gate");
              setServoStatus(1, 0, cGateMin);

              subModeIndex[0] = 5;

              break;

            // Check for green
            case 5:
              Serial.println("Search - Checking for Green");
              // Move to collection phase if green
              if (detectedGreen) {
                subModeIndex[1] = 0;
                robotModeIndex = 1;
              // Ignore gem
              } else {
                subModeIndex[0] = 6;
              }

              break;

            
            // Ignore gem then return to beginning
            case 6:
              encoderOffset = 400;
              if (absEncoderPos <= encoderOffset) {
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[0] = 7;
              }
              break;

            // Reverse a bit to give next gem room to trigger color sensor
            case 7:
              encoderOffset = 200;
              if (absEncoderPos <= encoderOffset) {
                setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos -= absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[0] = 0;
              }
              break;
              
          }
          break;

        // Collect gem
        case 1:
          switch (subModeIndex[1]) {
            
            // Raises arm to prepare for collection
            case 0:
              setServoStatus(0, 1500, cArmMax);

              subModeIndex[1] = 1;
              break;

            // Closes gate when gem inside collection chamber
            case 1:
              setServoStatus(1, 0, cGateMin);

              subModeIndex[1] = 2;
              break;
            
            // Collects gem
            case 2:
              setServoStatus(0, 0, cArmMin);

              subModeIndex[1] = 3;
              break;

            // Slowly moves arm to standby position
            case 3:
              Serial.println("Moving to standby");
              setServoStatus(0, 1500, cArmStandby);
              
              subModeIndex[1] = 4;
              break;

            case 4:
              Serial.println("Switching to verification mode");

              subModeIndex[2] = 0;
              robotModeIndex = 2;
              break;
          }
          break;
        
        // Verify gem collection
        case 2:
          switch (subModeIndex[2]) {

            //Drive forwards to clear color sensor
            case 0:
              encoderOffset = 100;
              if (absEncoderPos <= encoderOffset) {
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {

                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[2] = 1;
              }
              break;
            
            // Open gate for verification
            case 1:
              Serial.println("Verifying - Opening Gate");
              setServoStatus(1, 0, cGateMax);

              subModeIndex[2] = 2;
              break;

            // Drive backwards to reset position
            case 2:
              encoderOffset = 50;
              if (absEncoderPos <= encoderOffset) {
                setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {

                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos -= absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[2] = 3;
              }
              break;

            // Drive backwards for a set distance or until color sensor detects gem exiting
            case 3:
              Serial.println("Verifying - Detecting for Color");
              detectColor();
              encoderOffset = 600;
              
              if (!detectedGreen) {
                if (absEncoderPos <= encoderOffset) {
                  setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                  setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
                } else {
                  // No gem detected exiting, collection successful

                  setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                  setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                  totalEncoderPos -= absEncoderPos;
                  encoder[0].pos = 0;
                  encoder[1].pos = 0;

                  subModeIndex[2] = 5;
                }
              } else {
                // Gem detected exiting, collection failed
                subModeIndex[2] = 4;
              }
              break;
            

            // Collection failed, re-center gem for re-attempt
            case 4:
              Serial.println("Verifying - Collection Failed");
              encoderOffset = 250;
              if (absEncoderPos <= encoderOffset) {
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                subModeIndex[2] = 6;
              }
              break;

            // Collection successful, return to base
            case 5:
              Serial.println("Verifying - Collection Success");
              encoder[1].pos = 0;

              // Return to base
              subModeIndex[3] = 0;
              robotModeIndex = 3;
              break;

            case 6:
              Serial.println("Verified");

              encoderOffset = 150;
              if (absEncoderPos <= encoderOffset) {
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);

                totalEncoderPos += absEncoderPos;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                // Try to collect again
                subModeIndex[1] = 0;
                robotModeIndex = 1;
              }
              break;
          }
          break;
        
        // Return to base
        case 3:
          switch (subModeIndex[3]) {
            // Drive backwards until encoder position is the same as start
            case 0:
              if (absEncoderPos <= totalEncoderPos) {
                setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              } else {
                // No gem detected exiting, collection successful
                totalEncoderPos = 0;
                encoder[0].pos = 0;
                encoder[1].pos = 0;

                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
                subModeIndex[3] = 1;
              }
              break;
            
            // Returned to starting point, empty canister
            case 1:
              setServoStatus(0, 0, cArmMax);

              subModeIndex[3] = 2;
              break;

            // 2m is not up, try for another gem
            case 2:
            uint8_t timeUp = 120;
              if ((millis() - twoMinuteTimer)/1000 < timeUp) {
                // Try for another gem
                subModeIndex[0] = 0;
                robotModeIndex = 0;
              } else {
                // Stop robot
                setMotor(0, driveSpeed, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, driveSpeed, cIN1Pin[1], cIN2Pin[1]);
              }
              break;
          }
          break;
      }

      break;
  }
}

void setServo(int index) {
  uint32_t now = millis();

  // === Instant Move ===
  if (servoStatus[index].moveDuration == 0) {
    if (servoStatus[index].currentPos != servoStatus[index].targetPos) {
      servoStatus[index].currentPos = servoStatus[index].targetPos;
      servo[index].writeMicroseconds(servoStatus[index].currentPos);
    }
    return;
  }

  // === Timed Move ===
  uint32_t elapsed = now - servoStatus[index].startTime;
  if (elapsed >= servoStatus[index].moveDuration) {
    servoStatus[index].currentPos = servoStatus[index].targetPos;
    servo[index].writeMicroseconds(servoStatus[index].currentPos);
    return;
  }

  // Interpolate position
  uint16_t newPos = map(
    elapsed,
    0, servoStatus[index].moveDuration,
    servoStatus[index].startPos,
    servoStatus[index].targetPos
  );

  servoStatus[index].currentPos = newPos;
  servo[index].writeMicroseconds(servoStatus[index].currentPos);
}

void setServoStatus(int index, uint16_t moveDuration, uint16_t targetPos) {
  waitingForServo = true;
  waitStartTime = millis();

  servoStatus[index].moveDuration = moveDuration;
  servoStatus[index].targetPos = targetPos;
  servoStatus[index].startTime = waitStartTime;
  servoStatus[index].startPos = servoStatus[index].currentPos;
}

void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  } else if (dir == -1) {
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

void detectColor() {
  const float gGreenThreshold = 0.10;     // Green-specific threshold
  const float rgbThreshold = 0.25;        // General RGB detection threshold
  const float cClearThreshold = 0.30;     // Clear channel threshold

  if (tcsFlag) {
    tcs.getRawData(&r, &g, &b, &c);
    normalizeColor();
  }
    // --- Green Detection (G only with separate threshold) ---
  float gLow = baselineGreen[1] * (1 - gGreenThreshold);
  float gHigh = baselineGreen[1] * (1 + gGreenThreshold);
  bool gPass = (gNorm >= gLow && gNorm <= gHigh);
  detectedGreen = gPass;

  // --- General Color Detection (triggered if ANY channel is out of range) ---
  float rLow = baselineColor[0] * (1 - rgbThreshold);
  float rHigh = baselineColor[0] * (1 + rgbThreshold);
  bool rOut = (rNorm < rLow || rNorm > rHigh);

  float gLowGen = baselineColor[1] * (1 - rgbThreshold);
  float gHighGen = baselineColor[1] * (1 + rgbThreshold);
  bool gOut = (gNorm < gLowGen || gNorm > gHighGen);

  float bLow = baselineColor[2] * (1 - rgbThreshold);
  float bHigh = baselineColor[2] * (1 + rgbThreshold);
  bool bOut = (bNorm < bLow || bNorm > bHigh);

  float cLow = baselineColor[3] * (1.0 - cClearThreshold);
  float cHigh = baselineColor[3] * (1.0 + cClearThreshold);
  bool cOut = (c < cLow || c > cHigh);

  // Detected if any of the normalized RGB or raw C values are outside their ranges
  detectedColor = ((rOut || gOut || bOut) || cOut);

  detectedColor = detectedColor || detectedGreen;

  // --- Debug Output ---
  Serial.println("====== Color Detection Report ======");
  Serial.printf("Raw Sensor Values  ->  R: %d, G: %d, B: %d, C: %d\n", r, g, b, c);
  Serial.printf("Normalized Values   ->  R: %.3f, G: %.3f, B: %.3f\n", rNorm, gNorm, bNorm);
  Serial.println();

  // --- Green Detection Debug ---
  Serial.println("[Green Detection]");
  Serial.printf("G Baseline: %.3f, Threshold: ±%.3f -> %s\n",
                baselineGreen[1], gGreenThreshold, gPass ? "PASS" : "FAIL");
  Serial.printf("Green Detected: %s\n", detectedGreen ? "YES" : "NO");
  Serial.println();

  // --- General RGB + C Detection Debug ---
  Serial.println("[General Color Detection]");
  Serial.printf("R: %.3f [%.3f – %.3f] -> %s\n", rNorm, rLow, rHigh, rOut ? "OUT" : "IN");
  Serial.printf("G: %.3f [%.3f – %.3f] -> %s\n", gNorm, gLowGen, gHighGen, gOut ? "OUT" : "IN");
  Serial.printf("B: %.3f [%.3f – %.3f] -> %s\n", bNorm, bLow, bHigh, bOut ? "OUT" : "IN");
  Serial.printf("C: %d [%.1f – %.1f] -> %s\n", c, cLow, cHigh, cOut ? "OUT" : "IN");
  Serial.printf("Color Detected (RGB and C OUT of range or detected green): %s\n", detectedColor ? "YES" : "NO");
  Serial.println("=====================================\n");
}



void normalizeColor() {
  float total = (float)(r + g + b);
  rNorm = (float)r/total;
  bNorm = (float)b/total;
  gNorm = (float)g/total;
}


void doHeartbeat() {
  curMillis = millis();
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;
    LEDBrightnessIndex++;
    if (LEDBrightnessIndex >= sizeof(LEDBrightnessLevels)) {
      LEDBrightnessIndex = 0;
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);
    SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);
    SmartLEDs.show();
  }
}

void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);
  uint32_t pressTime = millis();
  if (pressTime > s->nextPressTime) {
    s->numberPresses += 1;
    s->pressed = true;
    s->nextPressTime = pressTime + cDebounceDelay;
  }
}

void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);
  int b = digitalRead(s->PinB);
  if (b > 0) {
    s->pos++;
  } else {
    s->pos--;
  }
}

