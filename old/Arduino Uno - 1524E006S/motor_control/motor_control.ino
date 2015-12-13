//  Arduino Mega
//  Macanum Base Velocity Control

//                  Front
//        FLW-M1  \\-----// FRW-M2
//                    |
//                    |
//                    |
//        RLW-M3  //-----\\ RRW-M4
//                   Back

const int encoder1A = 2, encoder2A = 0, encoder3A = 0, encoder4A = 0;               //Encoder A Channel
const int encoder1B = 3, encoder2B = 0, encoder3B = 0, encoder4B = 0;               //Encoder B Channel
const int PWM1 = 11, PWM2 = 0, PWM3 = 0, PWM4 = 0;                                  //PWM Pin
const int AIN1_1 = 23, AIN2_1 = 0, AIN3_1 = 0, AIN4_1 = 0;                          //Input 1 (DirectionSet)
const int AIN1_2 = 22, AIN2_2 = 0, AIN3_2 = 0, AIN4_2 = 0;                          //Input 2 (DirectionSet)
const int STBY1 = 24, STBY2 = 0, STBY3 = 0, STBY4 = 0;                              //Driver Standby

volatile long int interval1 = 0, interval2 = 0, interval3 = 0, interval4 = 0;       //RPM calculation interval
volatile long int timeold1 = 0, timeold2 = 0, timeold3 = 0, timeold4 = 0;           //RPM calculation
volatile long int timeoldPID = 0;                                                   //PID calculation
volatile int RPM1 = 0, RPM2 = 0, RPM3 = 0, RPM4 = 0;                                //RPM
volatile double velErr1 = 0, velErr2 = 0, velErr3 = 0, velErr4 = 0;                 //Velocity Error
volatile double velErrPrev1 = 0, velErrPrev2 = 0, velErrPrev3 = 0, velErrPrev4 = 0; //Previous Velocity Error
volatile double control1 = 0, control2 = 0, control3 = 0, control4 = 0;             //Control Signal
volatile double integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;         //PID -> integral
volatile double derivative1 = 0, derivative2 = 0, derivative3 = 0, derivative4 = 0; //PID -> derivative
volatile long int dt = 0;                                                           //Delta Time - PID           

double refVel1 = 0, refVel2 = 0, refVel3 = 0, refVel4 = 0;                          //Reference RPM
double Kp1 = 0.00025, Kp2 = 0.00025, Kp3 = 0.00025, Kp4 = 0.00025;                  //PID - Kp
double Ki1 = 0.0000001, Ki2 = 0.0000001, Ki3 = 0.0000001, Ki4 = 0.0000001;          //PID - Ki
double Kd1 = 0.0, Kd2 = 0.0, Kd3 = 0.0, Kd4 = 0.0;                                  //PID - Kd

int i = 0;

void setup() {  
  //set timer2 interrupt at 500Hz for PID loop
  TCCR2A = 0;                           // set entire TCCR2A register to 0
  TCCR2B = 0;                           // same for TCCR2B
  TCNT2  = 0;                           // initialize counter value to 0
  OCR2A = 249;                          // Set Compare match register for 500hz - 8bit Timer
  TCCR2A |= (1 << WGM21);               // Turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);  // Set CS2n bits for 128 prescaler
  TIMSK2 |= (1 << OCIE2A);              // Enable timer compare interrupt

  //Motor 1 - PWM frequency
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //Motor 2 - PWM frequency
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //Motor 3 - PWM frequency
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //Motor 4 - PWM frequency
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  
  pinMode(AIN1_2, OUTPUT), pinMode(AIN2_2, OUTPUT), pinMode(AIN3_2, OUTPUT), pinMode(AIN4_2, OUTPUT);         //Pololu Driver AIN2, direction2
  pinMode(AIN1_1, OUTPUT), pinMode(AIN2_1, OUTPUT), pinMode(AIN3_1, OUTPUT), pinMode(AIN4_1, OUTPUT);         //Pololu Driver AIN1, direction1
  pinMode(STBY1, OUTPUT), pinMode(STBY2, OUTPUT), pinMode(STBY3, OUTPUT), pinMode(STBY4, OUTPUT);             //Pololu Driver Standby mode
  pinMode(encoder1A, INPUT), pinMode(encoder2A, INPUT), pinMode(encoder3A, INPUT), pinMode(encoder4A, INPUT); //Encoder Channel A
  pinMode(encoder1B, INPUT), pinMode(encoder2B, INPUT), pinMode(encoder3B, INPUT), pinMode(encoder4B, INPUT); //Encoder Channel B 
  digitalWrite(STBY1, HIGH), digitalWrite(STBY2, HIGH), digitalWrite(STBY3, HIGH), digitalWrite(STBY4, HIGH); //Pololu Driver, Standby mode. LOW -> HIGH IMPEDENCE (STOP)

  attachInterrupt(0,encoder1A_RPM, RISING); //Motor-1 Encoder channel A interrupt
  attachInterrupt(1,encoder2A_RPM, RISING); //Motor-2 Encoder channel A interrupt
  attachInterrupt(2,encoder3A_RPM, RISING); //Motor-3 Encoder channel A interrupt
  attachInterrupt(3,encoder4A_RPM, RISING); //Motor-4 Encoder channel A interrupt
  Serial.begin(9600);
}

void loop() {
  while(true){
    Serial.println(RPM1);
    i++;
  }
}

//Motor-1 encoder channel A
void encoder1A_RPM(){
  interval1 = micros() - timeold1;
  timeold1 = micros();
  RPM1 = 60000000.0/interval1;
  if(digitalRead(encoder1B)==1){
    RPM1 = -RPM1;
  }
}

//Motor-2 encoder channel A
void encoder2A_RPM(){
  interval2 = micros() - timeold2;
  timeold2 = micros();
  RPM2 = 60000000.0/interval2;
  if(digitalRead(encoder2B)==1){
    RPM2 = -RPM2;
  }
}

//Motor-3 encoder channel A
void encoder3A_RPM(){
  interval3 = micros() - timeold3;
  timeold3 = micros();
  RPM3 = 60000000.0/interval3;
  if(digitalRead(encoder3B)==1){
    RPM3 = -RPM3;
  }
}

//Motor-4 encoder channel A
void encoder4A_RPM(){
  interval4 = micros() - timeold4;
  timeold4 = micros();
  RPM4 = 60000000.0/interval4;
  if(digitalRead(encoder4B)==1){
    RPM4 = -RPM4;
  }
}

//Timer2 interrupt at 500hz for PID
ISR(TIMER2_COMPA_vect){
  dt = micros() - timeoldPID;
  timeoldPID = micros();

  //PID - Motor 1
  velErr1 = refVel1 - RPM1;
  integral1 = integral1 + velErr1*dt;
  derivative1 = (velErr1 - velErrPrev1)/dt; 
  control1 = Kp1*velErr1 + Ki1*integral1 + Kd1*derivative1;
  velErrPrev1 = velErr1;

  //PID - Motor 2
  velErr2 = refVel2 - RPM2;
  integral2 = integral2 + velErr2*dt;
  derivative2 = (velErr2 - velErrPrev2)/dt; 
  control2 = Kp2*velErr2 + Ki2*integral2 + Kd2*derivative2;
  velErrPrev2 = velErr2;

  //PID - Motor 3
  velErr3 = refVel3 - RPM3;
  integral3 = integral3 + velErr3*dt;
  derivative3 = (velErr3 - velErrPrev3)/dt; 
  control3 = Kp3*velErr3 + Ki3*integral3 + Kd3*derivative3;
  velErrPrev3 = velErr3;

  //PID - Motor 4
  velErr4 = refVel4 - RPM4;
  integral4 = integral4 + velErr4*dt;
  derivative4 = (velErr4 - velErrPrev4)/dt; 
  control4 = Kp4*velErr4 + Ki4*integral4 + Kd4*derivative4;
  velErrPrev4 = velErr4;

  //Direction
  //AIN2 -> LOW, AIN1 -> HIGH, CW
  //AIN2 -> HIGH, AIN1 -> LOW, CCW
  if(control1 > 0){
    //CW
    digitalWrite(AIN1_2, LOW);
    digitalWrite(AIN1_1, HIGH);
  }
  else{
    //CCW
    digitalWrite(AIN1_2, HIGH);
    digitalWrite(AIN1_1, LOW);
  }

  if(control2 > 0){
    //CW
    digitalWrite(AIN2_2, LOW);
    digitalWrite(AIN2_1, HIGH);
  }
  else{
    //CCW
    digitalWrite(AIN2_2, HIGH);
    digitalWrite(AIN2_1, LOW);
  }

  if(control3 > 0){
    //CW
    digitalWrite(AIN3_2, LOW);
    digitalWrite(AIN3_1, HIGH);
  }
  else{
    //CCW
    digitalWrite(AIN3_2, HIGH);
    digitalWrite(AIN3_1, LOW);
  }

  if(control4 > 0){
    //CW
    digitalWrite(AIN4_2, LOW);
    digitalWrite(AIN4_1, HIGH);
  }
  else{
    //CCW
    digitalWrite(AIN4_2, HIGH);
    digitalWrite(AIN4_1, LOW);
  }
  
  //Saturate
  //Motor-1
  if(control1 > 255){
    control1 = 255;
  }
  if(control1 < -255){
    control1 = -255;
  }
  //Motor-2
  if(control2 > 255){
    control2 = 255;
  }
  if(control2 < -255){
    control2 = -255;
  }
  //Motor-3
  if(control3 > 255){
    control3 = 255;
  }
  if(control3 < -255){
    control3 = -255;
  }
  //Motor-4
  if(control4 > 255){
    control4 = 255;
  }
  if(control4 < -255){
    control4 = -255;
  }
  analogWrite(PWM1,abs((int)control1));
  analogWrite(PWM2,abs((int)control2));
  analogWrite(PWM3,abs((int)control3));
  analogWrite(PWM4,abs((int)control4));
}
