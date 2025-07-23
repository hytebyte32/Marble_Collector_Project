#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

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

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cNumMotors         = 2;                                      // Number of DC motors
const int cIN1Pin[]          = {26, 16};                               // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[]          = {27, 17};                               // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMRes            = 8;                                      // bit resolution for PWM
const int cMinPWM            = 150;                                    // PWM value for minimum speed that turns motor
const int cMaxPWM            = pow(2, cPWMRes) - 1;                    // PWM value for maximum speed
const int cPWMFreq           = 20000;                                  // frequency of PWM signal
const int cCountsRev         = 1096;                                   // encoder pulses per motor revolution
const int cPotPin            = 36;                                     // GPIO pin for drive speed potentiometer (A0)
const int cMotorEnablePin    = 39;                                     // GPIO pin for motor enable switch (DIP S1-1)
const int cTrigger           = 11;                                     // trigger duration in microseconds

// Variables
boolean motorsEnabled        = true;                                   // motors enabled flag
boolean timeUp3sec           = false;                                  // 3 second timer elapsed flag
boolean timeUp5sec           = false;                                  // 5 second timer elapsed flag
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t timerCount3sec      = 0;                                      // 3 second timer count in milliseconds
uint32_t timerCount5sec      = 0;                                      // 5 second timer count in milliseconds
uint32_t robotModeIndex      = 0;                                      // robot operational state                              
uint32_t lastTime            = 0;                                      // last time of motor control was updated
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
uint8_t driveSpeed           = 0;                                      // motor drive speed (0-255)
uint8_t driveIndex           = 0;                                      // state index for run mode
uint32_t cycles              = 0;                                      // motor cycle count, used to start US measurements

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

uint32_t modeIndicator[6]    = {                                       // colours for different modes
  SmartLEDs.Color(255, 0, 0),                                          //   red - stop
  SmartLEDs.Color(0, 255, 0),                                          //   green - run
  SmartLEDs.Color(0, 0, 255),                                          //   blue - empty case
  SmartLEDs.Color(255, 255, 0),                                        //   yellow - empty case
  SmartLEDs.Color(0, 255, 255),                                        //   cyan - empty case
  SmartLEDs.Color(255, 0, 255)                                         //   magenta - empty case
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

  // Set up push button
  pinMode(modeButton.pin, INPUT_PULLUP);                               // configure GPIO for mode button pin with internal pullup resistor
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING); // Configure ISR to trigger on low signal on pin
  
  pinMode(cPotPin, INPUT);                                             // set up drive speed potentiometer
  pinMode(cMotorEnablePin, INPUT);                                     // set up motor enable switch (uses external pullup)
}

void loop() {
  long pos[] = {0, 0};                                                 // current motor positions
  int pot = 0;                                                         // raw ADC value from pot
  uint32_t pulseDuration;                                              // duration of pulse read from HC-SR04
         
  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();                                                      // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                         // read and store current motor position
  }
  interrupts();                                                        // turn interrupts back on
 
  uint32_t curTime = micros();                                         // capture current time in microseconds
  if (curTime - lastTime > 1000) {                                     // wait 1 ms
    lastTime = curTime;                                                // update start time for next control cycle

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec += 1;                                               // increment 3 second timer count
    if (timerCount3sec > 3000) {                                       // if 3 seconds have elapsed
      timerCount3sec = 0;                                              // reset 3 second timer count
      timeUp3sec = true;                                               // indicate that 3 seconds have elapsed
    }  
   
    // 5 second timer, counts 2000 milliseconds
    timerCount5sec += 1;                                               // increment 5 second timer count
    if (timerCount5sec > 5000) {                                       // if 5 seconds have elapsed
      timerCount5sec = 0;                                              // reset 5 second timer count
      timeUp5sec = true;                                               // indicate that 5 seconds have elapsed
    }

    if (modeButton.pressed) {                                          // Change mode on button press
      robotModeIndex++;                                                // switch to next mode
      robotModeIndex = robotModeIndex & 7;                             // keep mode index between 0 and 7
      timerCount3sec = 0;                                              // reset 3 second timer count
      timeUp3sec = false;                                              // reset 3 second timer
      modeButton.pressed = false;                                      // reset flag
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(cMotorEnablePin);                     // if SW1-1 is on (low signal), then motors are enabled

    switch (robotModeIndex) {
      case 0: // Robot stopped
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor
        encoder[0].pos = 0;                                            // clear left encoder
        encoder[1].pos = 0;                                            // clear right encoder
        driveIndex = 0;                                                // reset drive index
        timerCount5sec = 0;                                            // reset 5 second timer count
        timeUp5sec = false;                                            // reset 5 second timer
        break;

      case 1: // Run robot
        if (timeUp3sec && motorsEnabled) {                             // pause for 3 sec before running case 1 code
                                                                       // and run only if enabled
          // Read pot to update drive motor speed
          pot = analogRead(cPotPin);
          driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
        #ifdef DEBUG_DRIVE_SPEED
                  Serial.printf("Drive Speed: Pot R1 = %d, mapped = %d\n", pot, driveSpeed);
        #endif
        #ifdef DEBUG_ENCODER_COUNTS
                  Serial.printf("Encoders: Left = %ld, Right = %ld\n", pos[0], pos[1]);
        #endif
        if (timeUp5sec) {                                            // update drive state after 5 seconds
          timerCount5sec = 0;                                        // reset 5 second timer count
          timeUp5sec = false;                                        // reset 5 second timer
          switch(driveIndex) {                                       // cycle through drive states
            case 0: // Stop
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);              // stop left motor
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);              // stop right motor
                driveIndex = 1;                                      // next state: drive forward
                break;

            case 1: // Drive forward — motors spin in opposite directions as they are opposed by 180 degrees
                setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);     // left motor forward
                setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);    // right motor reverse (opposite dir from left)
                driveIndex = 2;                                      // next state: drive backward
                break;

            case 2: // Drive backward — motors spin in opposite directions as they are opposed by 180 degrees
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);    // left motor reverse 
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);     // right motor forward (opposite dir from right)
                driveIndex = 3;                                      // next state: turn left
                break;

            case 3: // Turn left (counterclockwise) - motors spin in same direction
                setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);    // left motor reverse
                setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);    // right motor reverse
                driveIndex = 4;                                      // next state: turn right
                break;

            case 4: // Turn right (clockwise) — motors spin in same direction
                setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);     // left motor forward
                setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);     // right motor forward
                driveIndex = 0;                                      // next state: stop
                break;
          }
        }
      }
      else {                                                         // stop when motors are disabled
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                      // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                      // stop right motor
      }
      break;
    }
  }
  doHeartbeat();                                                       // update heartbeat LED
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