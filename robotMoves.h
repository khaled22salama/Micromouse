void startMotor() {
  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR2_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_B, LOW);
 // motorMoving = true;
}

void stopMotor() {
  digitalWrite(MOTOR1_A, LOW);
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_B, LOW);
  //motorMoving = false;
}

void turnLeft() {
  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_B, HIGH);
}
void turnRight() {
  digitalWrite(MOTOR1_A, LOW);
  digitalWrite(MOTOR2_A, HIGH);
  digitalWrite(MOTOR1_B, HIGH);
  digitalWrite(MOTOR2_B, LOW);
}

float getDistance(int counterx)
{
return ((counterx / 340.0) * CIRCUMFERENCE);
}