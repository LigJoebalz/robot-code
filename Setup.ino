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
