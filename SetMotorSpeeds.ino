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
