//Controller: Teensy 3.2
//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 9.28
//Mega Interrupt Pins: 2, 3, 18, 19, 20, 21

//#include <digitalWriteFast.h>
#include "pins.h"
#include "param.h"

IntervalTimer controlLoop;

//Motor-1 Constraints
volatile double RPM1 = 0;
volatile double refRPM1 = 0.0, errRPM1 = 0.0, integral1 = 0.0, derivative1 = 0.0, errPrevRPM1 = 0.0, control1 = 0.0;
volatile long int encPos1A = 0, encPos1B = 0, encPos1 = 0, dPos1 = 0, encPosOld1 = 0;

volatile long int dt, oldTime;

void setup() {
  noInterrupts();           // disable all interrupts

  pinMode(encoder1A_pin, INPUT);
  pinMode(encoder1B_pin, INPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN1_2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWM1, OUTPUT);

  digitalWriteFast(STBY, HIGH);    //Enable Motors

  analogWriteRes(9);   //9bit resolution PWM, 0-511
  
  attachInterrupt(5, encoder1A, RISING);
  attachInterrupt(6, encoder1B, RISING);
  
  Serial.begin(9600);
  analogWrite(PWM1, 128);
  digitalWriteFast(AIN1_1, HIGH);
  digitalWriteFast(AIN1_2, LOW);

  controlLoop.begin(control, contPeriod);
  
  interrupts();             // enable all interrupts
}

void loop(){
  Serial.print(dt);
  Serial.print(" ");
  Serial.print(encPos1A);
  Serial.print(" ");
  Serial.println(encPos1B);
  delay(100);
}

void encoder1A(){
  (digitalReadFast(encoder1B_pin)) ? encPos1A++ : encPos1A--;
}

void encoder1B(){
  (digitalReadFast(encoder1A_pin)) ? encPos1B-- : encPos1B++;
}

void control(){
  //RPM Calculation: 2-3uS
  //Motor-1 RPM Calculation
  encPos1 = (encPos1B + encPos1A)/2;          //Average of Encoder delta time(dt) A and B
  dPos1 = encPos1 - encPosOld1;               //Delta Position Encoder-1
  encPosOld1 = encPos1;                       //Update Position Old
  RPM1 = calcRPM(dPos1);                      //Motor-1 RPM
  
  //PID Calculation: 15-23uS
  //PID
  errRPM1 = refRPM1*gearRatio - RPM1; //4uS
  integral1 = integral1 + errRPM1*contPeriod; //4uS
  derivative1 = (errRPM1 - errPrevRPM1)/contPeriod; //10uS
  control1 = Kp1*errRPM1 + Ki1*integral1 + Kd1*derivative1; //6uS
  errPrevRPM1 = errRPM1;
  /*
  
  //Motor-1 Control
  if(control1 > 0){
    //CW Rotation
    digitalWriteFast(AIN1_1, HIGH);
    digitalWriteFast(AIN1_2, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(AIN1_1, LOW);
    digitalWriteFast(AIN1_2, HIGH);
  }
  
  
  //Motor-1 Control Saturate
  control1 = saturate(control1);
  
  analogWrite(PWM1, abs((int)control1));
  */
}

double calcRPM(int dPos){
  double RPM;
  RPM = (60000000.0/(contPeriod*encoderRes))*dPos;
  return RPM;
}

double saturate(double control){
  if(control > 511){
    control = 511;
  }
  if(control < -511){
    control = -511;
  }
  return control;
}

