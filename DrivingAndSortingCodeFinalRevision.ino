#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include <access.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"


#define DEBUG_DRIVE_SPEED    1
//#define DEBUG_ENCODER_COUNT  1
//#define STEP_OUTPUT_ON       1
//#define PRINT_COLOUR         1

void Indicator();                                                              // for mode/heartbeat on Smart LED
void updateEncoders();
void driveDistanceWithPID(int distance);
void turnLeft();
void turnRight();
float calculatePID(int pos);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR timerISR();

// Define motor pins and encoder pins
#define leftMotorIN1        35
#define leftMotorIN2        36
#define rightMotorIN1       37
#define rightMotorIN2       38
#define leftMotorEncoderPin1  15
#define leftMotorEncoderPin2  16
#define rightMotorEncoderPin1 11
#define rightMotorEncoderPin2 12
#define leftMotorPWM        6
#define rightMotorPWM       9
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use
#define IR_DETECTOR         13   

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;
// Constants Stepper
const int cPotPin            = 1;                      // when DIP switch S1-3 is on, pot (R1) is connected to GPIO1 (ADC1-0)
const int cToggle1           = 46;                     // DIP switch S1-2 turns stepper on/off  
const int cStepPin           = 40;                     // GPIO pin for step signal to A4988
const int cDirPin            = 39;                     // GPIO pin for direction signal to A4988
const int cStepRes           = 8;                      // bit resolution for stepper PWM stepper
const int cStepFreq          = 5;                    // initial frequency of stepper PWM
const long cDebounceDelay    = 20;
boolean runState             = false;                  // 0 = stopped; 1 = running
// Constants colour sensor
const int cSDA               = 47;                                             // GPIO pin for I2C data
const int cSCL               = 48;                                             // GPIO pin for I2C clock
const int cTCSLED            = 14;                                             // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;     
// Define constants for driving and turning
const int driveDistance = 15000;
const int leftTurnDuration = 20;
const int rightTurnDuration = 20;
// PID control constants and target value
const double kp = 0.9;
const double ki = 0.2;
const double kd = 0.1;
const int target = 900;
//Motor speed adjustment
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 3;  

// Define variables for PID control
int leftRightSwitch;
int ePrev;
float eInt;
unsigned long prevTime;
// Variables to track encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
//variables for main function
int robotModeIndex = 0;
int iteration = 0;
uint16_t r, g, b, c;                                // RGBC values from TCS34725

// Create servo objects for left and right motors
Servo leftMotor;
Servo rightMotor;
// Button structure
struct Button {
  const int pin;                                       // GPIO pin for button
  unsigned int numberPresses;                          // counter for number of button presses
  unsigned int lastPressTime;                          // time of last button press in ms
  bool pressed;                                        // flag for button press event
};
Button button                = {0, 0, 0, false};       // NO pushbutton PB1 on GPIO 0, low state when pressed
IR Scan = IR();                                                                // instance of IR for detecting IR signals
hw_timer_t * pTimer          = NULL;                   // pointer to timer used by timer interrupt
bool stepDir                 = true;                      // step direction
volatile int32_t stepCount   = 0;                      // number of steps
unsigned long curMillis      = 0;                                              // current time, in milliseconds
unsigned long prevMillis     = 0;   

Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);
// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};
unsigned int  modeIndicator[6] = {                                             // colours for different modes
   SmartLEDs.Color(255,0,0),                                                   //   red - stop
   SmartLEDs.Color(0,255,0),                                                   //   green - run
   SmartLEDs.Color(0,0,255),                                                   //   blue - empty case
   SmartLEDs.Color(255,255,0),                                                 //   yellow - empty case
   SmartLEDs.Color(0,255,255),                                                 //   cyan - empty case
   SmartLEDs.Color(255,0,255)                                                  //   magenta - empty case
};           
//=====================================================================================================================
// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found
 

void setup() {
  // Initialize serial communication if debugging is enabled
  #if defined DEBUG_DRIVE_SPEED || defined DEBUG_ENCODER_COUNT
    Serial.begin(115200);
  #endif

  // Attach servo objects to PWM pins
  leftMotor.attach(leftMotorPWM);
  rightMotor.attach(rightMotorPWM);

  // Set encoder pins as inputs
  pinMode(leftMotorEncoderPin1, INPUT);
  pinMode(leftMotorEncoderPin2, INPUT);
  pinMode(rightMotorEncoderPin1, INPUT);
  pinMode(rightMotorEncoderPin2, INPUT);

  // Set motor control pins as outputs
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorIN1, OUTPUT);
  pinMode(rightMotorIN2, OUTPUT);

  //Setup stepper
  pinMode(cToggle1, INPUT_PULLUP);                     // configure GPIO to turn stepper on/off 
  pinMode(cPotPin, INPUT);                             // configure potentiometer pin for input
  pinMode(cStepPin, OUTPUT);                           // assign output for step signal to A4988
  pinMode(cDirPin, OUTPUT);                            // assign output for direction signal to A4988

  pTimer = timerBegin(0, 80, true);                    // start timer 0 (1 of 4) with divide by 80 prescaler for 1 MHz resolution
                                                       // (see ESP32 Technical Reference Manual for more info).
  timerAttachInterrupt(pTimer, &timerISR, true);       // configure timer ISR
  timerAlarmWrite(pTimer, 500, true);                  // set initial interrupt time (in microseconds), set to repeat
  timerAlarmEnable(pTimer);                            // enable timer interrupt
  //colour sensor
  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 
  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }}

void loop() {
  runState=true;
  //switch to control between driving and searching for ir beacon
  switch(robotModeIndex){
    //drives in a sweeping motion across area until a specified number of passes have been made
    case 0:
      if(iteration > 5){
        robotModeIndex = 1;
      }else{
        iteration += 1;
        // Update encoder counts
        updateEncoders();
        // Drive forward, turn left, drive sideways, turn left, drive back, turn right, drive sideways, turn right (repeat)
        driveDistanceWithPID(driveDistance);
        Serial.println("1");
        turnLeft();
        Serial.println("1");
        driveDistanceWithPID(driveDistance);
        Serial.println("1");
        turnRight();
        Serial.println("1");

      }

    case 1:
      if(Scan.Available()){
        if (Scan.Get_IR_Data() == 'U'){
          driveDistanceWithPID(25);
        }else while(Scan.Get_IR_Data() != 'U'){
          setMotorSpeeds(1600, 1400);
          timerDelay(0.01);
          stopMotors();
      }
  }
}
}

// Function to read and update encoder counts
void updateEncoders() {
  // Read left motor encoder
  int leftEncoderState = digitalRead(leftMotorEncoderPin1) << 1 | digitalRead(leftMotorEncoderPin2);
  switch (leftEncoderState) {
    case 0b00:
    case 0b11:
      break; // No change or invalid state
    case 0b01:
    case 0b10:
      leftEncoderCount++;
      break; // Forward or backward movement
  }

  // Read right motor encoder
  int rightEncoderState = digitalRead(rightMotorEncoderPin1) << 1 | digitalRead(rightMotorEncoderPin2);
  switch (rightEncoderState) {
    case 0b00:
    case 0b11:
      break; // No change or invalid state
    case 0b01:
    case 0b10:
      rightEncoderCount++;
      break; // Forward or backward movement
  }
}

// Function to drive a specified distance using PID control
void driveDistanceWithPID(int distance) {
  // Reset encoder counts and store initial counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  long initialLeftCount = leftEncoderCount;
  long initialRightCount = rightEncoderCount;

  // Move until both motors reach the desired distance
  curMillis = millis();
  while (millis()<curMillis + distance) {
    // Calculate PID output for both motors
    double outputLeft = calculatePID(leftEncoderCount - initialLeftCount);
    double outputRight = calculatePID(rightEncoderCount - initialRightCount);
    checkObjectColor();

    // Apply PID output to motor speeds (limited to -500 to 500)
    setMotorSpeeds(constrain(1500 + outputLeft, 1000, 2000), constrain(1500 - outputRight, 1000, 2000));
   
  }
}

// Function to calculate PID output
float calculatePID(int pos) {
  // Calculate time since last PID calculation
  unsigned long currentTime = micros();
  float deltaTime = ((float)(currentTime - prevTime)) / 1.0e6;
  prevTime = currentTime;

  // Calculate error, derivative, and integral terms
  int e = pos - target;
  float dedt = (e - ePrev) / deltaTime;
  eInt = eInt + e * deltaTime;

  // Calculate PID control output
  float u = kp * e + kd * dedt + ki * eInt;
  u = constrain(u, -1000, 1000); // Limit control output

  // Update previous error for next iteration
  ePrev = e;

  return u;
}

// Function to set motor speeds and direction
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor.writeMicroseconds(leftSpeed);
  rightMotor.writeMicroseconds(rightSpeed);
  checkObjectColor();

  // Set motor direction based on speed
  if (leftSpeed > 1500) {
    digitalWrite(leftMotorIN1, HIGH);
    digitalWrite(leftMotorIN2, LOW);
  } else {
    digitalWrite(leftMotorIN1, LOW);
    digitalWrite(leftMotorIN2, HIGH);
  }

  if (rightSpeed > 1500) {
    digitalWrite(rightMotorIN1, HIGH);
    digitalWrite(rightMotorIN2, LOW);
  } else {
    digitalWrite(rightMotorIN1, LOW);
    digitalWrite(rightMotorIN2, HIGH);
  }
}

// Function to stop both motors
void stopMotors() {
  // Set PWM to neutral and input pins to LOW
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  digitalWrite(leftMotorIN1, LOW);
  digitalWrite(leftMotorIN2, LOW);
  digitalWrite(rightMotorIN1, LOW);
  digitalWrite(rightMotorIN2, LOW);
}

// Function to turn left
void turnLeft() {
  // Set one motor forward and the other backward for a left turn
  setMotorSpeeds(1600, -1400); // Adjust PWM values as needed
  timerDelay(leftTurnDuration);  // Adjust duration as needed
}

// Function to turn right
void turnRight() {
  // Set one motor forward and the other backward for a right turn
  setMotorSpeeds(-1400, 1600); // Adjust PWM values as needed
  timerDelay(rightTurnDuration);  // Adjust duration as needed
}
void checkObjectColor(){
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif

    // Set stepDir and runState based on dominant color
    if(c>20){ 
      if(g>b && g>r){
        stepDir = 1;
        runState = true;
      }else {
        stepDir = 0;
        runState = true;
      }
    }

    // Print stepDir and runState to serial monitor
    //Serial.printf("stepDir: %d, runState: %d\n", stepDir, runState);
  }

  //stepper functions
  int speedPot = analogRead(cPotPin);                                          // read speed pot value (between 0 and 4095)
  unsigned long stepRate = map(speedPot, 0, 4095, 500, 60000);                 // map to half period in microseconds
  if(stepDir){
    digitalWrite(cDirPin, 1);                                              // set direction pin
  }
  else{
    digitalWrite(cDirPin, 0);                                              // set direction pin
  }
}

void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);               // cast pointer to static structure

  uint32_t pressTime = millis();                       // capture current time
  if (pressTime - s->lastPressTime > cDebounceDelay) { // if enough time has passed to consider a valid press
    s->numberPresses += 1;                             // increment switch press counter
    s->pressed = true;                                 // indicate valid switch press state
    s->lastPressTime = pressTime;                      // update time to measure next press against
  }
}

// timer interrupt service routine
void ARDUINO_ISR_ATTR timerISR() {
  if (runState) {                                      // Only send pulse if motor should be running
    digitalWrite(cStepPin, !digitalRead(cStepPin));    // toggle state of step pin
    if (stepDir) {
      stepCount++;                                     // add to count in forward direction
    }
    else {
      stepCount--;                                     // subtract from count in reverse direction
    }
  }
}
void timerDelay(int elapsedTime){
  curMillis = millis();
  while(curMillis-prevMillis >= elapsedTime*1000){
    prevMillis = curMillis;
  }
}
