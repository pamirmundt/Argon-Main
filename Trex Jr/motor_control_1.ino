#include <SoftwareSerial.h>

#define TXPIN 4 //Trex Jr Tx
#define RXPIN 5 //Trex Jr Rx
#define encoder0PinA 2
#define encoder0PinB 3

volatile unsigned int encPosCount = 0;
double RPM = 0;
unsigned long int timeold = 0;
unsigned long int countold = 0;
unsigned long int interval = 0;


//PID Parameters
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

SoftwareSerial pololu(RXPIN, TXPIN);

void setup() {
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);
  
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(0, count, RISING); //encoder pin on interrupt 0 - pin 2
  
  Serial.begin(9600);
  pololu.begin(19200);
}

void loop() {
  pololu.write(0xC3); //set motor 1, 0xC0-0xC3 (C1->forward,C2->backward)
  pololu.write(127);  //set speed
  //Serial.println(encPosCount);
  interval = millis() - timeold;
  
  //Time interval = 5 sec
  if(interval >= 5000){
    RPM = (encPosCount-countold)*60*1000/interval;
    Serial.println(RPM/141); //Gear box ratio: 141:1
    countold = encPosCount;
    timeold = millis();
  }
}

void count(){
  /* If pinA and pinB are both high or both low, it is spinning
  * forward. If they're different, it's going backward.
  *
  * For more information on speeding up this process, see
  * [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encPosCount++;
  } else {
    encPosCount--;
  }
}
