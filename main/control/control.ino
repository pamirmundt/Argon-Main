//Controller: Arduino Mega
//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 9.28
//Mega Interrupt Pins: 2, 3, 18, 19, 20, 21

#include <digitalWriteFast.h>

#define encoderRes 344    //Encoder Resolution 344 pulse/rotation
#define gearRatio 9.28    //Gear Ratio 1:9.8

//Motor-1 Pins and constraints
const int encoder1A_pin = 2, encoder1B_pin = 3;   //Encoder Pins
const int PWM1 = 11;                              //PWM Pin
const int AIN1_1 = 22, AIN1_2 = 23;               //AIN Enable Pins
const int STBY1 = 24;                             //Standby Pin

volatile double RPM1 = 0;
volatile int dt1 = 0, dt1A = 0, dt1B = 0;
volatile long int timeOld1A = 0, timeOld1B = 0;
volatile long int encoderPos1A = 0, encoderPos1B = 0, encoderPos1 = 0;

const double Kp1 = 0.1, Ki1 = 0.0, Kd1 = 0.0;
volatile double refVel1 = 300.0, velErr1 = 0.0, control1 = 0.0;

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
  pinModeFast(STBY1, OUTPUT);
  pinModeFast(PWM1, OUTPUT);

  digitalWriteFast(STBY1, HIGH);    //Enable Motor-1
  digitalWriteFast(AIN1_1, LOW);
  digitalWriteFast(AIN1_2, HIGH);

  attachInterrupt(0, encoder1A, RISING);
  attachInterrupt(1, encoder1B, RISING);
  Serial.begin(9600);
  interrupts();             // enable all interrupts
}

void loop() {
  Serial.print(RPM1);
  Serial.print(" ");
  Serial.println(control1);
}

void encoder1A(){
  dt1A = micros() - timeOld1A;
  timeOld1A = micros();
  (digitalReadFast(encoder1B_pin) == 1) ? encoderPos1A-- : encoderPos1A++;
}

void encoder1B(){
  dt1B = micros() - timeOld1B;
  timeOld1B = micros();
  (digitalReadFast(encoder1A_pin) == 1) ? encoderPos1B++ : encoderPos1B--;
}

//RPM Calculation
ISR(TIMER2_COMPA_vect){
  //Motor-1 RPM Calculation
  dt1 = (dt1A + dt1B)/2;  //Average of Encoder delta time(dt) A and B
  RPM1 = calcRPM(dt1);        //Motor-1 RPM

  //PID
  velErr1 = refVel1 - RPM1;
  control1 = Kp1*velErr1 + control1;
  /*
  //Motor-1 Control
  if(control1 > 0){
    //CW Rotation
      digitalWriteFast(AIN1_1, LOW);
      digitalWriteFast(AIN1_2, HIGH);
  } else {
    //CCW Rotation
      digitalWriteFast(AIN1_1, HIGH);
      digitalWriteFast(AIN1_2, LOW);
  }
  */
  //Motor-1 Control Saturate
  if(control1 > 255){
    control1 = 255;
  }
  if(control1 < -255){
    control1 = -255;
  }
  analogWrite(PWM1, control1);
}

double calcRPM(int dt){
  double RPM;
  if (dt > 0)
    RPM = 60000000.0/(dt*gearRatio*encoderRes);
  else
    RPM = 0.0;
  return RPM;
}

