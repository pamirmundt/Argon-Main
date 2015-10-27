const int encoderA = 2; //Encoder A Channel
const int encoderB = 3; //Encoder B Channel
const int PWMA = 5;     //PWM A Pin
const int AIN2 = 6;     //Channel A - Input 2
const int AIN1 = 7;     //Channel A - Input 1
const int STBY = 8;     //Driver Standby

volatile long int interval = 0;   //RPM calculation interval
volatile long int timeold = 0;    //RPM calculation
volatile int RPM = 0;             //RPM
volatile double velErr = 0;       //Velocity Error
volatile double control = 0;      //Control Signal
double refVel = 12000;            //Reference RPM
double Kp = 0.00025;              //PID - Kp
double Ki = 0.0;                  //PID - Ki
double Kd = 0.0;                  //PID - Kd

volatile long int interval2 = 0;
volatile long int timeold2 = 0;

int i = 0;

void setup() {
  //set timer2 interrupt at 500Hz
  TCCR2A = 0;                           // set entire TCCR2A register to 0
  TCCR2B = 0;                           // same for TCCR2B
  TCNT2  = 0;                           //initialize counter value to 0
  OCR2A = 249;                          //Set Compare match register for 500hz - 8bit Timer
  TCCR2A |= (1 << WGM21);               //Turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);  //Set CS2n bits for 128 prescaler
  TIMSK2 |= (1 << OCIE2A);              //Enable timer compare interrupt

  //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
  //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  
  pinMode(AIN2, OUTPUT);    //Pololu Driver AIN2, direction2
  pinMode(AIN1, OUTPUT);    //Pololu Driver AIN1, direction1
  pinMode(STBY, OUTPUT);    //Pololu Driver Standby mode
  pinMode(encoderA, INPUT); //Encoder Channel A
  pinMode(encoderB, INPUT); //Encoder Channel B 
  digitalWrite(STBY, HIGH); //Pololu Driver, Standby mode. LOW -> HIGH IMPEDENCE (STOP)

  attachInterrupt(0,encoderA_RPM, RISING); //Encoder channel A interrupt
  Serial.begin(9600);
}

void loop() {
  //analogWrite(PWMA, 73); //Max 255 - 6V
  while(true){
    Serial.println(interval2);
    i++;
  }
}

void encoderA_RPM(){
  interval = micros() - timeold;
  timeold = micros();
  RPM = 60000000.0/interval;
  if(digitalRead(encoderB)==1){
    RPM = -RPM;
  }
}

//Timer2 interrupt at 500hz for PID
ISR(TIMER2_COMPA_vect){
  interval2 = micros() - timeold2;
  timeold2 = micros();
  velErr = refVel - RPM;
  control = Kp*velErr + control;
  
  //AIN2 -> LOW, AIN1 -> HIGH, CW
  //AIN2 -> HIGH, AIN1 -> LOW, CCW
  if(control > 0){
    //CW
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
  }
  else{
    //CCW
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
  }  
  if(control > 255){
    control = 255;
  }
  if(control < -255){
    control = -255;
  }
  analogWrite(PWMA,abs((int)control));  
}

