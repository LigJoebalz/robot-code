// timer interrupt service routine
void ARDUINO_ISR_ATTR timerISR() {
  if (runState) {                                      // Only send pulse if motor should be running
    digitalWrite(cStepPin, !digitalRead(cStepPin));    // toggle state of step pin
    if (stepDir) {
      stepCount++;                                     // add to count in forward direction
    }
    else {
      stepCount--;                                     // subtract from count in reverse direction
    }
  }
}
