//Controller: Teensy 3.2

#include "basePins.h"
#include "baseMotor.h"

//TO DO
//- PID windup check
//- SetMotorPWM class

IntervalTimer controlInterrupt1;
IntervalTimer controlInterrupt2;
IntervalTimer controlInterrupt3;
IntervalTimer controlInterrupt4;

BMotor motor[4];  //Base Motors

float test = 50;

void setup() {
  //Clean BMotor
  memset(&motor, 0, sizeof(BMotor));

  motor[FRONT_LEFT_WHEEL].mEnable = true;
  motor[FRONT_RIGHT_WHEEL].mEnable = true;
  motor[REAR_LEFT_WHEEL].mEnable = true;
  motor[REAR_RIGHT_WHEEL].mEnable = true;
  
  motor[FRONT_LEFT_WHEEL].refRPM = test;
  motor[FRONT_RIGHT_WHEEL].refRPM = test;
  motor[REAR_LEFT_WHEEL].refRPM = test;
  motor[REAR_RIGHT_WHEEL].refRPM = test;
  
  //disable all interrupts
  noInterrupts();                   
  
  //Pololu Driver-1 (Motor 1-2)
  pinMode(STBY1, OUTPUT); 
  //Pololu Driver-2 (Motor 3-4)
  pinMode(STBY2, OUTPUT);
  //Motor-1 Pins
  pinMode(encoder1A_pin, INPUT);
  pinMode(encoder1B_pin, INPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN1_2, OUTPUT);
  pinMode(PWM1_pin, OUTPUT);
  //Motor-2 Pins
  pinMode(encoder2A_pin, INPUT);
  pinMode(encoder2B_pin, INPUT);
  pinMode(BIN1_1, OUTPUT);
  pinMode(BIN1_2, OUTPUT);
  pinMode(PWM2_pin, OUTPUT);
  //Motor-3 Pins
  pinMode(encoder3A_pin, INPUT);
  pinMode(encoder3B_pin, INPUT);
  pinMode(AIN2_1, OUTPUT);
  pinMode(AIN2_2, OUTPUT);
  pinMode(PWM3_pin, OUTPUT);
  //Motor-4 Pins
  pinMode(encoder4A_pin, INPUT);
  pinMode(encoder4B_pin, INPUT);
  pinMode(BIN2_1, OUTPUT);
  pinMode(BIN2_2, OUTPUT);
  pinMode(PWM4_pin, OUTPUT);
  //Enable Motors 1-2
  digitalWriteFast(STBY1, HIGH);
  //Enable Motors 3-4
  digitalWriteFast(STBY2, HIGH);   
  //9-bit resolution PWM, 0-511, 93750 Hz
  analogWriteRes(9);   

  if(motor[0].mEnable){
    attachInterrupt(encoder1A_pin, encoder1A, RISING);            //Motor-1 Encoder Interrupt
    controlInterrupt1.begin(controlLoop1, (contPerSec*1000000));  //Motor-1 Control Interrupt
  }
  if(motor[1].mEnable){
    attachInterrupt(encoder2A_pin, encoder2A, RISING);            //Motor-2 Encoder Interrupt
    controlInterrupt2.begin(controlLoop2, (contPerSec*1000000));  //Motor-2 Control Interrupt
  }
  if(motor[2].mEnable){
    attachInterrupt(encoder3A_pin, encoder3A, RISING);            //Motor-3 Encoder Interrupt
    controlInterrupt3.begin(controlLoop3, (contPerSec*1000000));  //Motor-3 Control Interrupt
  }
  if(motor[3].mEnable){
    attachInterrupt(encoder4A_pin, encoder4A, RISING);            //Motor-4 Encoder Interrupt
    controlInterrupt4.begin(controlLoop4, (contPerSec*1000000));  //Motor-4 Control Interrupt
  }
  
  interrupts();       //Enable all interrupts
  Serial.begin(9600); //Start Serial
} 

void loop(){
  Serial.print(motor[0].posEnc);
  Serial.print(" ");
  Serial.print(motor[1].posEnc);
  Serial.print(" ");
  Serial.print(motor[2].posEnc);
  Serial.print(" ");
  Serial.println(motor[3].posEnc);
  delay(250);
}

/*****Motor Encoders*****/
//Motor-1 Encoder
void encoder1A(){
  (digitalReadFast(encoder1B_pin)) ? motor[0].posEnc++ : motor[0].posEnc--;
}

//Motor-2 Encoder
void encoder2A(){
  (digitalReadFast(encoder2B_pin)) ? motor[1].posEnc++ : motor[1].posEnc--;
}

//Motor-3 Encoders
void encoder3A(){
  (digitalReadFast(encoder3B_pin)) ? motor[2].posEnc++ : motor[2].posEnc--;
}

//Motor-4 Encoders
void encoder4A(){
  (digitalReadFast(encoder4B_pin)) ? motor[3].posEnc++ : motor[3].posEnc--;
}

/*****Interrupt-1*****/
/*****M1  Control*****/
void controlLoop1(){
  calcRPM(&motor[0]);
  calcPID(&motor[0]);
  setMotor1PWM(motor[0].control);
}

/*****Interrupt-2*****/
/*****M2  Control*****/
void controlLoop2(){
  calcRPM(&motor[1]);
  calcPID(&motor[1]);
  setMotor2PWM(motor[1].control);
}

/*****Interrupt-3*****/
/*****M3  Control*****/
void controlLoop3(){
  calcRPM(&motor[2]);
  calcPID(&motor[2]);
  setMotor3PWM(motor[2].control);
}

/*****Interrupt-4*****/
/*****M4  Control*****/
void controlLoop4(){
  calcRPM(&motor[3]);
  calcPID(&motor[3]);
  setMotor4PWM(motor[3].control);
}

void calcPID(BMotor * m){
  //Int-PID, 2^12 Scale
  m->errRPM = ((int)(m->refRPM * 4)) * ((int)(gearRatio * 128)) - m->RPM;
  m->integral =  m->integral + ((m->errRPM * ((int)(round(contPerSec * 4096)))) >> 12);
  m->derivative = ((m->errRPM - m->errPrevRPM) << 3) / ((int)(round(contPerSec * 4096)));
  m->control = (((int)(round(Kp * 4096))) * m->errRPM + ((int)(round(Ki * 4096))) * m->integral + ((int)(round(Kd * 4096))) * m->derivative) >> 21;
  m->errPrevRPM = m->errRPM;
  
  /*
  //Float-PID
  errRPM1 = refRPM1*gearRatio - RPM1;
  integral1 = integral1 + errRPM1*contPerSec;
  derivative1 = (errRPM1 - errPrevRPM1)/contPerSec;
  control1 = Kp1*errRPM1 + Ki1*integral1 + Kd1*derivative1;
  errPrevRPM1 = errRPM1;
  */
}

void calcRPM(BMotor * m){    
  m->dPos = m->posEnc - m->posEncPrev;                  //Delta Position Encoder-1           
  m->posEncPrev = m->posEnc;                            //Update Position Old                  
  m->RPM = 9197*m->dPos;                          //RPM << 9 - 2^9 Scale
  
  //Float RPM Calculation
  //RPM = ((60000000.0)/(contPeriod*encoderRes))*dPos;
}

void setMotor1PWM(int control){
  control = constrain(control, -511, 511);
  if(control > 0){
    //CW Rotation
    digitalWriteFast(AIN1_1, HIGH);
    digitalWriteFast(AIN1_2, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(AIN1_1, LOW);
    digitalWriteFast(AIN1_2, HIGH);
  }
  analogWrite(PWM1_pin, abs(control));
}

void setMotor2PWM(int control){
  control = constrain(control, -511, 511);
  if(control > 0){
    //CW Rotation
    digitalWriteFast(BIN1_1, HIGH);
    digitalWriteFast(BIN1_2, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(BIN1_1, LOW);
    digitalWriteFast(BIN1_2, HIGH);
  }
  analogWrite(PWM2_pin, abs(control));
}

void setMotor3PWM(int control){
  control = constrain(control, -511, 511);
  if(control > 0){
    //CW Rotation
    digitalWriteFast(AIN2_1, HIGH);
    digitalWriteFast(AIN2_2, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(AIN2_1, LOW);
    digitalWriteFast(AIN2_2, HIGH);
  }
  analogWrite(PWM3_pin, abs(control));
}

void setMotor4PWM(int control){
  control = constrain(control, -511, 511);
  if(control > 0){
    //CW Rotation
    digitalWriteFast(BIN2_1, HIGH);
    digitalWriteFast(BIN2_2, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(BIN2_1, LOW);
    digitalWriteFast(BIN2_2, HIGH);
  }
  analogWrite(PWM4_pin, abs(control));
}
