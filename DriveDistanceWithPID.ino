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
   
    timerDelay(1); // Adjust delay as needed for PID control frequency
  }
}
