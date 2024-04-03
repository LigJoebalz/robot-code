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
