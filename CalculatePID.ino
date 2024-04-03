// Function to calculate PID output
float calculatePID(int pos) {
  // Calculate time since last PID calculation
  unsigned long currentTime = micros();
  float deltaTime = ((float)(currentTime - prevTime)) / 1.0e6;
  prevTime = currentTime;

  // Calculate error, derivative, and integral terms
  int e = pos - target;
  float dedt = (e - ePrev) / deltaTime;
  eInt = eInt + e * deltaTime;

  // Calculate PID control output
  float u = kp * e + kd * dedt + ki * eInt;
  u = constrain(u, -1000, 1000); // Limit control output

  // Update previous error for next iteration
  ePrev = e;

  return u;
}
