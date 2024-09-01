#include <PID_v1.h>

float getSpeed(long lastTime,int pulseMotor)
{
  float motorSpeed;
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 1000) {  // Calculate speed every second
    //noInterrupts();
    motorSpeed = (float)pulseMotor / 11;  // Assuming 11 pulses per revolution
    
    //interrupts();

    Serial.print("Motor Speed (RPM): ");
    Serial.println(motorSpeed);

    lastTime = currentTime;
    return motorSpeed;
  }
}