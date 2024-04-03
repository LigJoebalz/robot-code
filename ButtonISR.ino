// Interuptable Service Routine for Button Presses
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);               // cast pointer to static structure

  uint32_t pressTime = millis();                       // capture current time
  if (pressTime - s->lastPressTime > cDebounceDelay) { // if enough time has passed to consider a valid press
    s->numberPresses += 1;                             // increment switch press counter
    s->pressed = true;                                 // indicate valid switch press state
    s->lastPressTime = pressTime;                      // update time to measure next press against
  }
}
