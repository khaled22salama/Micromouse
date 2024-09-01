#include "robot_config.h"
#include "motorPID.h"
#include "robotMoves.h"


//******************** PID variable **********************************
double speed;  // will be the desired valuem (speed || WALLs)

//------------Motor 1 PID----------------
double Input1;                                                    // US sensor
double Output1;                                                   //Motors
double Kp1 = 0, Ki1 = 0, Kd1 = 0;                                 //PID parameters
PID motor1PID(&Input1, &Output1, &speed, Kp1, Ki1, Kd1, DIRECT);  //create PID instance
float motor1Speed = 0.0;
volatile int counter1 = 0;


//------------Motor 2 PID؛----------------
double Input2;                                                    // US sensor
double Output2;                                                   //Motors
double Kp2 = 0, Ki2 = 0, Kd2 = 0;                                 //PID parameters
PID motor2PID(&Input2, &Output2, &speed, Kp2, Ki2, Kd2, DIRECT);  //create PID instance
float motor2Speed = 0.0;
volatile int counter2 = 0;


//--------------------------------------------------------------------------------
unsigned long lastTime = 0;  // Last time reading
volatile int pulseMotor1, pulseMotor2;


/***********************************************************************************/
float distance1 = 0;
float distance2 = 0;



// Counter for the number of encoder pulses
bool motorMoving = false;

void setup() {
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  stopMotor();

  pinMode(ENCODER_PIN, INPUT);
  pinMode(MAGNETIC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);  // Attach interrupt to the encoder pin
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN), encoder2ISR, RISING);

  Serial.begin(9600);

  motor1PID.SetMode(AUTOMATIC);         //Turn the PID on
  motor1PID.SetTunings(Kp1, Ki1, Kd1);  //Adjust PID values

  motor2PID.SetMode(AUTOMATIC);         //Turn the PID on
  motor2PID.SetTunings(Kp2, Ki2, Kd2);  //Adjust PID values
}

void loop() {

  distance1 = getDistance(counter1);
  distance2 = getDistance(counter2);


  if (distance1 > (-1.0 * TURNDISTANCE) && distance2 > (-1.0 * TURNDISTANCE))  //turnRight
    turnRight();
  else {
    stopMotor();
  }
  /*-------------------------------------------------------------------------------------------*/
  if (distance1 < (TURNDISTANCE) && distance2 < (TURNDISTANCE))  //turnLeft
    turnLeft();
  else {
    stopMotor();
  }
  /*-------------------------------------------------------------------------------------------*/






  Input1 = getSpeed(lastTime,pulseMotor1);
  motor1PID.Compute();     //PID calculation
  analogWrite(3, Output1);  //Write the output as calculated by the PID function
  pulseMotor1 = 0;                       // Reset pulse count

  Input2 = getSpeed(lastTime,pulseMotor2);
  motor2PID.Compute();     //PID calculation
  analogWrite(3, Output2);  //Write the output as calculated by the PID function
  pulseMotor2 = 0;                       // Reset pulse count

  //Send data by serial for plotting
  /*Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);
  Serial.print(" ");  
  Serial.println(speed);*/
  //  delay(100);
}

void encoderISR() {
  if (digitalRead(MAGNETIC_PIN) == HIGH) {
    counter1--;
    pulseMotor1--;
  } else {
    counter1++;
    pulseMotor1++;
  }
}
void encoder2ISR() {
  if (digitalRead(MAGNETIC2_PIN) == HIGH)
    {counter2--;
  pulseMotor2--;}

  else {counter2++;
  pulseMotor2--;}
}
