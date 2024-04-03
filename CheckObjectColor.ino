// checks the color of any object located within the feed chamber of the robot and, based upon its color determination, it chooses to either reject it, or store it in its launch chamber.
void checkObjectColor(){
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif

    // Set stepDir and runState based on dominant color
    if(c>20){ 
      if(g>b && g>r){
        stepDir = 1;
        runState = true;
      }else {
        stepDir = 0;
        runState = true;
      }
    }

    // Print stepDir and runState to serial monitor
    Serial.printf("stepDir: %d, runState: %d\n", stepDir, runState);
  }

  //stepper functions
  int speedPot = analogRead(cPotPin);                                          // read speed pot value (between 0 and 4095)
  unsigned long stepRate = map(speedPot, 0, 4095, 500, 60000);                 // map to half period in microseconds
  if(stepDir){
    digitalWrite(cDirPin, 1);                                              // set direction pin
  }
  else{
    digitalWrite(cDirPin, 0);                                              // set direction pin
  }
}
