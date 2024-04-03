#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include <access.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"


#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1
#define STEP_OUTPUT_ON       1
#define PRINT_COLOUR         1

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
const int driveDistance = 200;
const int carWidth = 10;
const int leftTurnDuration = 30;
const int rightTurnDuration = 30;
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
 
