// Function to turn right
void turnRight() {
  // Set one motor forward and the other backward for a right turn
  setMotorSpeeds(1400, 1600); // Adjust PWM values as needed
  timerDelay(rightTurnDuration);  // Adjust duration as needed

  // Stop motors after turning
  stopMotors();
}
