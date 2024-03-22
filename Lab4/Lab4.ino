#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Function declarations
// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use
#define IR_DETECTOR         14                                                 // GPIO14 pin 17 (J14) IR detector input
#define SHOULDER_SERVO      41                                                 // GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO          42                                                 // GPIO42 pin 35 (J42) Servo 2

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 0;                                                    // Amount to slow down right motor relative to left
const int driveDistance = 15000;
const int turnDistance = 1200;
const int overShoot = 500;
// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;                                                 // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned int potClawSetpoint;                                                  // desired position of claw servo read from pot
unsigned int potShoulderSetpoint;                                              // desired position of shoulder servo read from pot
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

// Declare SK6812 SMART LED object
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

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data
IR Scan = IR();                                                                // instance of IR for detecting IR signals

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
   Serial.begin(115200);
#endif
   
  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder
   
   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop() {
  long pos[] = {0, 0};                                                        // current motor positions
  int pot = 0;                                                                // raw ADC value from pot
  // check if drive motors should be powered
  motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled
  pot = analogRead(POT_R1);
  leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
  rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;

  if (motorsEnabled) {                                            // run motors only if enabled
                                                              // update drive state after 2 seconds
    switch(driveIndex) {                                      // cycle through drive states
      case 0: // Stop
        Bot.Stop("D1");                                     // drive ID
        driveIndex++; 
        currentMicros = millis(); 
        while(millis() < currentMicros + overShoot){
        Serial.println('Waiting');
        }                             
        break;

      case 1: // Drive forward
        Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
        driveIndex++; 
        currentMicros = millis(); 
        while(millis() < currentMicros + driveDistance){
        Serial.println('Waiting');
        }                                      
        break;

      case 2: // Turn left
        Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);    // drive ID, left speed, right speed
        driveIndex++;
        currentMicros = millis(); 
        while(millis() < currentMicros + turnDistance){
        Serial.println('Waiting');
        }                                             
        break;

      case 3: // Drive forward
        Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
        driveIndex++;  
        currentMicros = millis(); 
        while(millis() < currentMicros + overShoot){
        Serial.println('Waiting');
        }                                              
        break;

      case 4: // Turn left
        Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);    // drive ID, left speed, right speed
        driveIndex++;  
        currentMicros = millis(); 
        while(millis() < currentMicros + turnDistance){
        Serial.println('Waiting');
        }                                           
        break;

      case 5: // Drive forward
        Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
        driveIndex++;    
        currentMicros = millis(); 
        while(millis() < currentMicros + driveDistance){
        Serial.println('Waiting');
        }                                            
        break;

      case 6: // Turn right
        Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);   // drive ID, left speed, right speed
        driveIndex++;          
        currentMicros = millis(); 
        while(millis() < currentMicros + turnDistance){
        Serial.println('Waiting');
        }                         
        break;

      case 7: // Drive forward
        Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
        driveIndex++;           
        currentMicros = millis(); 
        while(millis() < currentMicros + overShoot){
        Serial.println('Waiting');
        }                                     
        break;

      case 8: // Turn right
        Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);   // drive ID, left speed, right speed
        driveIndex = 0;    
        currentMicros = millis(); 
        while(millis() < currentMicros + turnDistance){
        Serial.println('Waiting');
        }                               
        break;
    }
  }
  
  LEDBrightnessIndex++;                                                // shift to next brightness level
  if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {              // if all defined levels have been used
    LEDBrightnessIndex = 0;                                           // reset to starting brightness
  }
  SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // set brightness of heartbeat LED
}