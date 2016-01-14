//Controller: Teensy 3.2
//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 9.28

//TODO
//- Motor class

//#include <digitalWriteFast.h>
#include "pins.h"
#include "param.h"

IntervalTimer controlInterrupt1;

//Motor-1 Constraints
volatile float refRPM1 = 0.0;
volatile int32_t RPM1 = 0, errRPM1 = 0, integral1 = 0, derivative1 = 0, errPrevRPM1 = 0, control1 = 0;
volatile int32_t encPos1A = 0, encPos1B = 0, encPos1 = 0, dPos1 = 0, encPosOld1 = 0;

void setup() {
  noInterrupts();                   // disable all interrupts
  //Pololu Driver-1 (Motor 1-2)
  pinMode(STBY1, OUTPUT);
  //Motor-1 Pins
  pinMode(encoder1A_pin, INPUT);
  pinMode(encoder1B_pin, INPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN1_2, OUTPUT);
  pinMode(PWM1, OUTPUT);

  //Motor-2 Pins
  pinMode(encoder2A_pin, INPUT);
  pinMode(encoder2B_pin, INPUT);
  pinMode(AIN2_1, OUTPUT);
  pinMode(AIN2_2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  digitalWriteFast(STBY1, HIGH);    //Enable Motors 1-2

  analogWriteRes(9);   //9-bit resolution PWM, 0-511, 93750 Hz
  
  //Motor Encoder Interrupts
  attachInterrupt(encoder1A_pin, encoder1A, RISING);  //Motor-1
  attachInterrupt(encoder1B_pin, encoder1B, RISING);  //Motor-1
  //attachInterrupt();
  //attachInterrupt();

  //Motor Control Interrupts
  controlInterrupt1.begin(controlLoop1, (contPerSec*1000000));
  
  Serial.begin(9600); //Start Serial
  interrupts();       //Enable all interrupts
} 

void loop(){
  Serial.print(encPos1A);
  Serial.print(" ");
  Serial.print(encPos1B);
  Serial.print(" ");
  Serial.println(RPM1/512/gearRatio);
  delay(250);
}

void encoder1A(){
  (digitalReadFast(encoder1B_pin)) ? encPos1A++ : encPos1A--;
}

void encoder1B(){
  (digitalReadFast(encoder1A_pin)) ? encPos1B-- : encPos1B++;
}

void controlLoop1(){
  //Motor-1 RPM Calculation
  encPos1 = (encPos1B + encPos1A) >> 1;       //Average of Encoder delta time(dt) A and B
  dPos1 = encPos1 - encPosOld1;               //Delta Position Encoder-1
  encPosOld1 = encPos1;                       //Update Position Old
  RPM1 = calcRPM(dPos1);                      //Motor-1 RPM  
  
  //Motor1 Int-PID, 2^12 Scale
  errRPM1 = ((int)(refRPM1*4))*((int)(gearRatio*128)) - RPM1;
  integral1 =  integral1 + ((errRPM1*((int)(round(contPerSec*4096)))) >> 12);
  derivative1 = ((errRPM1 - errPrevRPM1) << 3)/((int)(round(contPerSec*4096)));
  control1 = (((int)(round(Kp1*4096)))*errRPM1 + ((int)(round(Ki1*4096)))*integral1 + ((int)(round(Kd1*4096)))*derivative1) >> 21;
  errPrevRPM1 = errRPM1;
  
  /*
  //Float-PID
  errRPM1 = refRPM1*gearRatio - RPM1;
  integral1 = integral1 + errRPM1*contPerSec;
  derivative1 = (errRPM1 - errPrevRPM1)/contPerSec;
  control1 = Kp1*errRPM1 + Ki1*integral1 + Kd1*derivative1;
  errPrevRPM1 = errRPM1;
  */
  
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
  analogWrite(PWM1, abs(control1));
}

int32_t calcRPM(int32_t dPos){
  int32_t RPM;
  RPM = 9197*dPos;  //RPM << 9 - 2^9 Scale
  //Float RPM Calculation
  //RPM = ((60000000.0)/(contPeriod*encoderRes))*dPos;
  return RPM;
}

int32_t saturate(int32_t control){
  if(control > 511 || control < -511){
    control = 511;
  }
  return control;
}

