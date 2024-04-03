void loop() {
  runState=true;
  //switch to control between driving and searching for ir beacon
  switch(robotModeIndex){
    //drives in a sweeping motion across area until a specified number of passes have been made
    case 0:
      if(iteration > 10){
        robotModeIndex = 1;
      }else{
        iteration += 1;
        // Update encoder counts
        updateEncoders();
        // Drive forward, turn left, drive sideways, turn left, drive back, turn right, drive sideways, turn right (repeat)
        checkObjectColor();       
        driveDistanceWithPID(driveDistance);
        checkObjectColor();
        turnLeft();
        checkObjectColor();
        driveDistanceWithPID(carWidth);
        checkObjectColor();
        turnLeft();
        checkObjectColor();
        driveDistanceWithPID(driveDistance);
        checkObjectColor();
        turnRight();
        checkObjectColor();
        driveDistanceWithPID(carWidth);
        checkObjectColor();
        turnRight();
      }

    case 1:
      if(Scan.Available()){
        if (Scan.Get_IR_Data() == 'U'){
          driveDistanceWithPID(25);
        }else while(Scan.Get_IR_Data() != 'U'){
          setMotorSpeeds(1600, 1400);
          timerDelay(0.01);
          stopMotors();
      }
  }
}
}
