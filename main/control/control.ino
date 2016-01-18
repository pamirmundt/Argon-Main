//Controller: Teensy 3.2

#include "basePins.h"
#include "baseMotor.h"

IntervalTimer controlInterrupt1;

BMotor motor[2];  //Base Motors

void setup() {
  memset(&motor, 0, sizeof(BMotor));
  motor[FRONT_LEFT_WHEEL].refRPM = 100;
  //disable all interrupts
  noInterrupts();                   
  
  //Pololu Driver-1 (Motor 1-2)
  pinMode(STBY1, OUTPUT); 
  //Motor-1 Pins
  pinMode(encoder1A_pin, INPUT);
  pinMode(encoder1B_pin, INPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN1_2, OUTPUT);
  pinMode(PWM1_pin, OUTPUT);
  //Motor-2 Pins
  pinMode(encoder2A_pin, INPUT);
  pinMode(encoder2B_pin, INPUT);
  pinMode(AIN2_1, OUTPUT);
  pinMode(AIN2_2, OUTPUT);
  pinMode(PWM2_pin, OUTPUT);
  //Enable Motors 1-2
  digitalWriteFast(STBY1, HIGH);    
  //9-bit resolution PWM, 0-511, 93750 Hz
  analogWriteRes(9);   
  
  //Motor Encoder Interrupts
  attachInterrupt(encoder1A_pin, encoder1A, RISING);  //Motor-1
  attachInterrupt(encoder1B_pin, encoder1B, RISING);  //Motor-1
  
  //Motor Control Interrupts
  controlInterrupt1.begin(controlLoop1, (contPerSec*1000000));
  
  interrupts();       //Enable all interrupts
  Serial.begin(9600); //Start Serial
} 

void loop(){
  Serial.print(motor[0].encPosA);
  Serial.print(" ");
  Serial.print(motor[0].encPosB);
  Serial.print(" ");
  Serial.println(motor[0].RPM/512/gearRatio);
  delay(250);
}

void encoder1A(){
  (digitalReadFast(encoder1B_pin)) ? motor[0].encPosA++ : motor[0].encPosA--;
}

void encoder1B(){
  (digitalReadFast(encoder1A_pin)) ? motor[0].encPosB-- : motor[0].encPosB++;
}

/*****Interrupt-1*****/
/*****PID Control*****/
void controlLoop1(){
  //Motor-1
  calcRPM(&motor[0]);
  calcPID(&motor[0]);
  setMotor1PWM(motor[0].control);
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
  m->pos = (m->encPosA + m->encPosB) >> 1;        //Average of Encoder delta time(dt) A and B      
  m->dPos = m->pos - m->posPrev;                  //Delta Position Encoder-1           
  m->posPrev = m->pos;                            //Update Position Old                  
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
