//Controller: Arduino Mega
//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 9.28
//Mega Interrupt Pins: 2, 3, 18, 19, 20, 21

#include <digitalWriteFast.h>
#include "pins.h"
#include "param.h"

//Motor-1 Constraints
volatile double RPM1 = 0;
volatile double refRPM1 = 0.0, errRPM1 = 0.0, integral1 = 0.0, derivative1 = 0.0, errPrevRPM1 = 0.0, control1 = 0.0;
volatile long int encPos1A = 0, encPos1B = 0, encPos1 = 0, dPos1 = 0, encPosOld1 = 0;

void setup() {
  noInterrupts();           // disable all interrupts
  
  //set timer2 interrupt at 500Hz for RPM calculation loop
  TCCR2A = 0;                           // set entire TCCR2A register to 0
  TCCR2B = 0;                           // same for TCCR2B
  TCNT2  = 0;                           // initialize counter value to 0
  OCR2A = 249;                          // Set Compare match register for 500hz - 8bit Timer
  TCCR2A |= (1 << WGM21);               // Turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);  // Set CS2n bits for 128 prescaler
  TIMSK2 |= (1 << OCIE2A);              // Enable timer compare interrupt
  
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz

  pinModeFast(encoder1A_pin, INPUT);
  pinModeFast(encoder1B_pin, INPUT);
  pinModeFast(AIN1_1, OUTPUT);
  pinModeFast(AIN1_2, OUTPUT);
  pinModeFast(STBY, OUTPUT);
  pinModeFast(PWM1, OUTPUT);

  digitalWriteFast(STBY, HIGH);    //Enable Motors

  attachInterrupt(0, encoder1A, RISING);
  attachInterrupt(1, encoder1B, RISING);
  
  Serial.begin(9600);
  
  interrupts();             // enable all interrupts
}

void loop(){
  Serial.println(RPM1/gearRatio);
}

void encoder1A(){
  (digitalReadFast(encoder1B_pin)) ? encPos1A-- : encPos1A++;
}

void encoder1B(){
  (digitalReadFast(encoder1A_pin)) ? encPos1B++ : encPos1B--;
}

//Control Loop
ISR(TIMER2_COMPA_vect){
  //Motor-1 RPM Calculation
  encPos1 = (encPos1B + encPos1A)/2;          //Average of Encoder delta time(dt) A and B
  dPos1 = encPos1 - encPosOld1;               //Delta Position Encoder-1
  encPosOld1 = encPos1;                       //Update Position Old
  RPM1 = calcRPM(dPos1);                      //Motor-1 RPM
  
  //PID
  errRPM1 = refRPM1*gearRatio - RPM1;
  integral1 = integral1 + errRPM1*contPer;
  derivative1 = (errRPM1 - errPrevRPM1)/contPer;
  control1 = Kp1*errRPM1 + Ki1*integral1 + Kd1*derivative1;
  errPrevRPM1 = errRPM1;

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
}

double calcRPM(int dPos){
  double RPM;
  RPM = (60000000.0/(contPer*encoderRes))*dPos;
  return RPM;
}

double saturate(double control){
  if(control > 255){
    control = 255;
  }
  if(control < -255){
    control = -255;
  }
  return control;
}

