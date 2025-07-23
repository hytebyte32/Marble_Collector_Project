#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Button structure
struct Button {
  const int pin;                                                       // GPIO pin for button
  volatile uint32_t numberPresses;                                     // counter for number of button presses
  uint32_t nextPressTime;                                              // time of next allowable press in milliseconds
  volatile bool pressed;                                               // flag for button press event
};

// Encoder structure
struct Encoder {
  const int PinA;                                                      // GPIO pin for encoder Channel A
  const int PinB;                                                      // GPIO pin for encoder Channel B
  volatile long pos;                                                   // current encoder position
};

// Switch structure
struct Switch {
  const int pin;                                      // GPIO pin for switch
  uint32_t numberPresses;                             // counter for number of switch presses
  uint32_t nextPressTime;                             // time of next allowable press in milliseconds
  bool pressed;                                       // state variable to indicate "valid" switch press
};

// Servo structure
struct Servo {
  const int pin;                                      // GPIO pin for servo
  uint16_t currentPos;                                 // records current servo position
  uint16_t targetPos;                                  // records servo target position
};

// Function declarations
bool calculateColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
bool detectColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t baseline[]);
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void setServo(Servo &servo);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR switchISR(void* arg);

// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cNumMotors         = 2;                                      // Number of DC motors
const int cPWMRes            = 8;                                      // bit resolution for PWM
const int cMinPWM            = 150;                                    // PWM value for minimum speed that turns motor
const int cMaxPWM            = pow(2, cPWMRes) - 1;                    // PWM value for maximum speed
const int cArmMax            = 2048;                                   // Max duty for arm servo
const int cArmMin            = 955;                                    // Min duty for arm servo
const int cGateMax           = 2048;                                   // Max duty for gate servo
const int cGateMin           = 409;                                    // Min duty for gate servo
const int cPWMFreq           = 20000;                                  // frequency of PWM signal
const int cCountsRev         = 1096;                                   // encoder pulses per motor revolution
const int cPotPin            = 36;                                     // GPIO pin for drive speed potentiometer (A0)
const int cSDA               = 18;                                     // GPIO pin for I2C data
const int cSCL               = 19;                                     // GPIO pin for I2C clock
const int cTCSLED            = 14;                                     // GPIO pin for LED on TCS34725 
const int cIN1Pin[]          = {26, 16};                               // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[]          = {27, 17};                               // GPIO pin(s) for IN2 for left and right motors (A, B)
int cMotorAdjustment[]       = {0, 0};                                 // PWM adjustment for motors to run closer to the same speed

// Variables
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t robotModeIndex      = 0;                                      // robot operational state               
uint32_t subModeIndex[]      = {0, 0, 0, 0, 0};
uint32_t lastModeIndex       = 0;                                      // last robot operational state                 
uint32_t lastTime            = 0;                                      // last time of motor control was updated
uint32_t absEncoderPos       = 0;                                      // absolute average encoder position of both motors
uint32_t totalEncoderPos     = 0;                                      // total distance travelled
uint16_t baselineColor[]     = {0, 0, 0, 0};                           // records the baseline rgbc values of the floor
uint16_t r, g, b, c;                                                   // RGBC values from TCS34725
uint16_t encoderOffset        = 0;                                      // how much the motors should move additionally for positioning
uint8_t driveSpeed           = 0;                                      // motor drive speed (0-255)
uint8_t startButton          = 0;                                      // start button
bool detectedGreen           = false;                                  // true if detected color is green
bool detectedColor           = false;                                  // true if any color is detected


Switch limitSwitch           = {15, 0, 0, false};                      // initialize limit switch
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
Servo servo[]                = {{4, 955, 955},                         // Arm servo, starting in the raised position
                                {13, 409, 409}};                       // Gate servo, starting in the closed position


Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 flag: 1 = connected; 0 = not found
bool tcsFlag = 0;                                                      

uint32_t modeIndicator[6]    = {                                       // colours for different modes
  SmartLEDs.Color(255, 0, 0),                                          //   red - stop
  SmartLEDs.Color(0, 255, 0),                                          //   green - run
  SmartLEDs.Color(0, 0, 255),                                          //   blue - ultrasonic
  SmartLEDs.Color(255, 255, 0),                                        //   yellow - IR detector
  SmartLEDs.Color(0, 255, 255),                                        //   cyan - claw servo
  SmartLEDs.Color(255, 0, 255),                                        //   magenta - arm servo
};

void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                                   // initialize smart LEDs object
  SmartLEDs.clear();                                                   // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0));                  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                                          // set brightness [0-255]
  SmartLEDs.show();                                                    // update LED

  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);                         // setup INT1 GPIO PWM Channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);                         // setup INT2 GPIO PWM Channel
    pinMode(encoder[k].PinA, INPUT);                                   // configure GPIO for encoder Channel A input
    pinMode(encoder[k].PinB, INPUT);                                   // configure GPIO for encoder Channel B input

    // configure encoder to trigger interrupt with each rising edge on Channel A
    attachInterruptArg(encoder[k].PinA, encoderISR, &encoder[k], RISING);
  }

  // Set up servos
  pinMode(servo[0].pin, OUTPUT);                                        // configure arm servo GPIO for output
  pinMode(servo[1].pin, OUTPUT);                                        // configure gate servo GPIO for output
  ledcAttach(servo[0].pin, 50, 14);                                     // setup arm servo pin for 50Hz and 14-bit resolution
  ledcAttach(servo[1].pin, 50, 14);                                     // setup arm servo pin for 50Hz and 14-bit resolution
  ledcWrite(servo[0].pin, servo[0].currentPos);                         // initialize arm servo position
  ledcWrite(servo[1].pin, servo[1].currentPos);                         // initialize gate servo position

  // Set up push button
  pinMode(modeButton.pin, INPUT_PULLUP);                                // configure GPIO for mode button pin with internal pullup resistor
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING);  // configure ISR to trigger on low signal on pin

  pinMode(limitSwitch.pin, INPUT_PULLUP); 
  attachInterruptArg(limitSwitch.pin, switchISR, &limitSwitch, FALLING);
  
  pinMode(cPotPin, INPUT);                                              // set up drive speed potentiometer

  Wire.setPins(cSDA, cSCL);                                             // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                                             // configure GPIO to control LED on TCS34725

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  // Gather baseline rgbc values
  digitalWrite(cTCSLED, true);                                          // turn on onboard LED
  uint8_t sampleSize = 10;
  for (int i = 0; i < sampleSize; i++) {
    if (tcsFlag) {                                                       // if colour sensor initialized
      tcs.getRawData(&r, &g, &b, &c);                                    // get raw RGBC values
      baselineColor[0] += r;
      baselineColor[1] += g;
      baselineColor[2] += b;
      baselineColor[3] += c;
    }
  }
  baselineColor[0] /= sampleSize;
  baselineColor[1] /= sampleSize;
  baselineColor[2] /= sampleSize;
  baselineColor[3] /= sampleSize;
  digitalWrite(cTCSLED, false);                                          // turn off onboard LED
}

void loop() {
  long pos[] = {0, 0};                                                 // current motor positions
  int pot = 0;                                                         // raw ADC value from pot

  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();                                                      // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                         // read and store current motor position
  }
  interrupts();                                                        // turn interrupts back on

  if (modeButton.pressed) {                                          // Change mode on button press
    startButton++;                                                // switch to next mode
    startButton = startButton & 1;                                   // keep mode index between 0 and 1
    modeButton.pressed = false;                                      // reset flag
    if (startButton) {
      // Reset everything
      robotModeIndex = 0;
      subModeIndex[0] = 0;
      subModeIndex[1] = 0;
      subModeIndex[2] = 0;
      subModeIndex[3] = 0;
      subModeIndex[4] = 0;
      totalEncoderPos = 0;
      encoder[0].pos = 0;
      encoder[1].pos = 0;
    }
  }

  // Read pot to update drive motor speed
  pot = analogRead(cPotPin);
  driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);

  absEncoderPos = (abs(pos[0]) + abs(pos[1]))/2;
  doHeartbeat();                                                       // update heartbeat LED
}

bool calculateColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  uint16_t r_range[] = {6, 10};
  uint16_t g_range[] = {6, 11};
  uint16_t b_range[] = {5, 8};
  uint16_t c_range[] = {20, 30};

  return (r >= r_range[0] && r <= r_range[1]) &&
         (g >= g_range[0] && g <= g_range[1]) &&
         (b >= b_range[0] && b <= b_range[1]) &&
         (c >= c_range[0] && c <= c_range[1]);
}

bool detectColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t baseline[]) {
  const float threshold = 0.10;                                        // 10% change allowed

  return (abs((int)r - baseline[0]) > baseline[0] * threshold) ||
         (abs((int)g - baseline[1]) > baseline[1] * threshold) ||
         (abs((int)b - baseline[2]) > baseline[2] * threshold) ||
         (abs((int)c - baseline[3]) > baseline[3] * threshold);
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                                                // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                         // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                              // shift to the next brightness level
    if (LEDBrightnessIndex >= sizeof(LEDBrightnessLevels)) {           // if all defined levels have been used
      LEDBrightnessIndex = 0;                                          // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);         // set pixel colors to = mode 
    SmartLEDs.show();                                                  // update LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                                      // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                                                // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                                               // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// moves servo incrementally to make it trackable
void setServo(Servo &servo) {
  if(servo.currentPos > servo.targetPos) {
    servo.currentPos--;
    ledcWrite(servo.pin, servo.currentPos);
  } else if (servo.currentPos < servo.targetPos) {
    servo.currentPos++;
    ledcWrite(servo.pin, servo.currentPos);
  }
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, 
// allowing multiple instances of the buttonISR to be created (1 per button)
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);                               // cast pointer to static structure

  uint32_t pressTime = millis();                                       // capture current time
  if (pressTime > s->nextPressTime) {                                  // if enough time has passed to consider a valid press
    s->numberPresses += 1;                                             // increment button press counter
    s->pressed = true;                                                 // indicate valid button press state
    s->nextPressTime = pressTime + cDebounceDelay;                     // update time for next valid press
  }  
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);                             // cast pointer to static structure
  
  int b = digitalRead(s->PinB);                                        // read state of Channel B
  if (b > 0) {                                                         // high, leading Channel A
    s->pos++;                                                          // increase position
  }
  else {                                                               // low, lagging Channel A
    s->pos--;                                                          // decrease position
  }
}

void ARDUINO_ISR_ATTR switchISR(void* arg) {
  Switch* s = static_cast<Switch*>(arg);                               // cast pointer to static structure

  uint32_t pressTime = millis();                                       // capture current time
  if (pressTime > s->nextPressTime) {                                  // if enough time has passed to consider a valid press
    s->numberPresses += 1;                                             // increment button press counter
    s->pressed = true;                                                 // indicate valid button press state
    s->nextPressTime = pressTime + cDebounceDelay;                     // update time for next valid press
  }
}