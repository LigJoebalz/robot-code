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
