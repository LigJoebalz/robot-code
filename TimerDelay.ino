void timerDelay(int elapsedTime){
  curMillis = millis();
  while(curMillis-prevMillis >= elapsedTime*1000){
    prevMillis = curMillis;
  }
}
