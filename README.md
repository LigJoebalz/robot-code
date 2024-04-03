# Arduino Motor Control Project

This project involves controlling motors, implementing PID control, using encoder feedback, integrating servo motors, and sensor interaction on an Arduino board. The Complete Code is available in the `DrivingAndSortingCode.ino` file.


## Components Used
- Arduino board
- Adafruit NeoPixel library
- MSE2202_Lib library
- Adafruit TCS34725 color sensor library
- Servo library

## Setup
1. Connect the motors to the designated pins (`leftMotorIN1`, `leftMotorIN2`, `rightMotorIN1`, `rightMotorIN2`).
2. Attach encoders to the encoder pins (`leftMotorEncoderPin1`, `leftMotorEncoderPin2`, `rightMotorEncoderPin1`, `rightMotorEncoderPin2`).
3. Connect servo motors to PWM pins (`leftMotorPWM`, `rightMotorPWM`).
4. Configure other components as specified in the code.

## Functions
- `Indicator()`: Controls mode/heartbeat on the Smart LED.
- `updateEncoders()`: Reads and updates encoder counts.
- `driveDistanceWithPID(int distance)`: Drives a specified distance using PID control.
- `turnLeft()`, `turnRight()`: Turns the robot left or right.
- `calculatePID(int pos)`: Calculates PID output for motor control.
- `setMotorSpeeds(int leftSpeed, int rightSpeed)`: Sets motor speeds and direction.
- `stopMotors()`: Stops both motors.
- `buttonISR(void* arg)`, `timerISR()`: Interrupt service routines for button and timer events.
- `checkObjectColor()`: Checks color sensor data and adjusts motor behavior accordingly.
- `timerDelay(int elapsedTime)`: Delays program execution based on elapsed time.

## Usage
1. Upload the code to your Arduino board.
2. Ensure all connections are correct and functional.
3. Monitor serial output for debugging and status information.
4. Interact with the system using buttons or sensors as specified in the code.

## Project Overview
This Arduino project focuses on controlling a robotic system that includes motor control, PID (Proportional-Integral-Derivative) control for precise movement, encoder feedback for position tracking, integration of servo motors for additional functionality, and interaction with a color sensor for object detection or color-based decision-making.

The main functionalities of the code are as follows:
1. **Motor Control:** The code controls the movement of motors connected to the Arduino board, allowing the robot to move forward, turn left or right, and perform specific maneuvers.
2. **PID Control:** It implements PID control algorithms to ensure smooth and accurate motor movement, especially over specified distances or angles.
3. **Encoder Feedback:** The code reads encoder values to track the position and movement of the robot, enabling precise navigation and control.
4. **Servo Motor Integration:** Servo motors are utilized for additional functionalities such as controlling specific parts of the robot, like arms or grippers.
5. **Color Sensor Interaction:** The code interacts with a color sensor to detect and respond to different colors or objects, enabling the robot to perform tasks based on color recognition.

The project's overall goal is to create a robotic system capable of controlled and intelligent movement, with the ability to make decisions based on sensor inputs such as color detection. It combines hardware components and software algorithms to achieve a functional and interactive robotic platform.
