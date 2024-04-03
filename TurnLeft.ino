// Function to turn left
void turnLeft() {
  // Set one motor forward and the other backward for a left turn
  setMotorSpeeds(1600, 1400); // Adjust PWM values as needed
  timerDelay(leftTurnDuration);  // Adjust duration as needed

  // Stop motors after turning
  stopMotors();
}
