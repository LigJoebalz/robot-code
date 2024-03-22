#include <Arduino.h>
#include <Servo.h>

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

// Define constants for driving and turning
int driveDistance = 200;
int carWidth = 10;
int leftTurnDuration = 30;
int rightTurnDuration = 30;

// PID control constants and target value
double kp = 0.5;
double ki = 0.2;
double kd = 0.1;
int target = 900;

// Define variables for PID control
int ePrev;
float eInt;
unsigned long prevTime;

// Create servo objects for left and right motors
Servo leftMotor;
Servo rightMotor;

// Variables to track encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Function prototypes
void updateEncoders();
void driveDistanceWithPID(int distance);
void turnLeft();
void turnRight();
float calculatePID(int pos);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();

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
}

void loop() {
  // Update encoder counts
  updateEncoders();

  // Drive forward, turn left, drive sideways, turn left, drive back, turn right, drive sideways, turn right (repeat)
  driveDistanceWithPID(driveDistance);
  turnLeft();
  driveDistanceWithPID(carWidth);
  turnLeft();
  driveDistanceWithPID(driveDistance);
  turnRight();
  driveDistanceWithPID(carWidth);
  turnRight();
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
  while (abs(leftEncoderCount - initialLeftCount) < distance || abs(rightEncoderCount - initialRightCount) < distance) {
    // Calculate PID output for both motors
    double outputLeft = calculatePID(leftEncoderCount - initialLeftCount);
    double outputRight = calculatePID(rightEncoderCount - initialRightCount);

    // Apply PID output to motor speeds (limited to -500 to 500)
    setMotorSpeeds(constrain(1500 + outputLeft, 1000, 2000), constrain(1500 - outputRight, 1000, 2000));
   
    delay(10); // Adjust delay as needed for PID control frequency
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
  setMotorSpeeds(1600, 1400); // Adjust PWM values as needed
  delay(leftTurnDuration);  // Adjust duration as needed

  // Stop motors after turning
  stopMotors();
}

// Function to turn right
void turnRight() {
  // Set one motor forward and the other backward for a right turn
  setMotorSpeeds(1400, 1600); // Adjust PWM values as needed
  delay(rightTurnDuration);  // Adjust duration as needed

  // Stop motors after turning
  stopMotors();
}