//Master as Arduino Mega
//Timer0 - 8Bit
//Timer1 - 16Bit
//Timer2 - 8Bit
//Timer3 - 16Bit
//Timer4 - 16Bit
//Timer5 - 16Bit

#include <Wire.h>
#include "msgs.h"

//Set Configuration Commands
#define CMD_RESET 0x02
#define CMD_MOVE_SPEED 0x03
#define CMD_MOVE_TO 0x04
#define CMD_SET_PID 0x05
#define CMD_SET_POWER 0x06
#define CMD_SET_PWM 0x07
#define CMD_SET_MODE 0x08

//Get Motor Status
#define CMD_GET_PID 0x20        //32
#define CMD_GET_POWER 0x21      //33
#define CMD_GET_POS 0x22        //34
#define CMD_GET_SPEED 0x23      //35
#define CMD_GET_PWM 0x24        //36
#define CMD_GET_REF_SPEED 0x25  //37

//Enable-Disable Status Messages(msgs)
#define CMD_GET_MSGS 0x30       //48

//Initialize I2C Buffer Size 
const uint8_t dataSize = 26;
byte bufI2C[dataSize];
byte rdata[dataSize];

//Initialize messages(msgs)
MSGS msgs;
boolean msgsEnabled = false; //Default

void setup() {
  msgs.msgsFreq = MSGS_FREQ;  //Set Messages Frequency (see msgs.h for frequency)

  //Initialize Serial and I2C(Master Mode)
  //Address: 0x00
  Serial.begin(9600);
  Wire.begin(0);

  //MSGS Timer
  timersInit();
}

void loop() {

  //Listen Serial for Commands
  while(Serial.available()){
    disableMsgs();
    int cmd = Serial.parseInt();
    cmd_parser(cmd);
  }
  
  //Print msgs
  if(msgsEnabled)
    printMsgs();
}

void cmd_parser(uint8_t cmd){
  // msgs ->    │Wheel(uint16)│EncPos(int32)│RPM(float)│PWM(float)│RefPos(int32)│RefRPM(int32)│
  //                ControlFreq(float)│MsgsFreq(float)│
  // bufI2C ->  │CMD(uint8_t)│Param1(float)│Param2(float)│Param3(float)│
  
  //Disable Msgs Dialog before cmd parsing
  disableMsgs();

  //Set the first byte of bufI2C as cmd
  memcpy(&bufI2C[0], &cmd, sizeof(cmd));

  //CMD Parser
  switch(cmd){
    //***** SET CONFIG *****
    case CMD_RESET: //2
      Serial.println("CMD_RESET(2)");
      I2C_send(bufI2C, sizeof(bufI2C));
      break;
    
    case CMD_MOVE_SPEED:  //3
      //Parameters: Velocity
      Serial.println("CMD_MOVE_SPEED (3)");
      readParams(1);
      I2C_send(bufI2C, sizeof(bufI2C));
      enableMsgs();
      break;
      
    case CMD_MOVE_TO:     //4
      Serial.println("CMD_MOVE_TO (4)");
      readParams(1);
      I2C_send(bufI2C, sizeof(bufI2C));
      break;
      
    case CMD_SET_PID:     //5
      Serial.println("CMD_SET_PID (5)");
      readParams(3);
      I2C_send(bufI2C, sizeof(bufI2C));
      break;

    case CMD_SET_POWER:    //6
      Serial.println("CMD_SET_POWER (6)");
      readParams(1);
      I2C_send(bufI2C, sizeof(bufI2C));
      break;
      
    case CMD_SET_PWM:     //7
      Serial.println("CMD_SET_PWM (7)");
      readParams(1);
      I2C_send(bufI2C, sizeof(bufI2C));
      break;

    case CMD_SET_MODE:    //8
      Serial.println("CMD_SET_MODE");
      readParams(1);
      I2C_send(bufI2C, sizeof(bufI2C));
      break;

    //***** GET STATUS *****
    case CMD_GET_PID: {   //32
      float params[] = {0, 0, 0};
      I2C_send(bufI2C, sizeof(bufI2C));
      I2C_request(params, 3);
      Serial.print("Kp: ");
      Serial.println(params[0]);
      Serial.print("Ki: ");
      Serial.println(params[1]);
      Serial.print("Kd: ");
      Serial.println(params[2]);
    } break;

    case CMD_GET_POWER:     //33
    case CMD_GET_SPEED:     //35
    case CMD_GET_PWM:       //36
    case CMD_GET_REF_SPEED: //37
    {
      float params[] = {0};
      I2C_send(bufI2C, sizeof(bufI2C));
      I2C_request(params, 1);
      Serial.println(params[0]);
    } break;

    case CMD_GET_POS:       //34
    {
      float params[] = {0};
      I2C_send(bufI2C, sizeof(bufI2C));
      I2C_request(params, 1);

      //***DÜZELTİLECEK***
      int32_t paramsInt[] = {0};
      memcpy(&paramsInt[0], &rdata[0], sizeof(int32_t));
      
      Serial.println(paramsInt[0]);
    } break;

    case CMD_GET_MSGS:       //48
    {
      msgsEnabled = !msgsEnabled;
    }  break;
  }

  //Enable msgs display again if necessary
  if(msgsEnabled)
    enableMsgs(); 
}

void I2C_send(byte* data, uint8_t pSize){
  Wire.beginTransmission(1);
  Wire.write(data, pSize);
  Wire.endTransmission();
}

void readParams(uint8_t paramNum){
  uint8_t index = 0;
  float p[paramNum];
  
  while(Serial.available() && (index < paramNum)){
    p[index] = Serial.parseFloat();
    memcpy(&bufI2C[(index*4+1)],&p[index], sizeof(float));
    index++;
  }
}

void msgsRequestParser(){
    //uint8_t msgsCMD = CMD_GET_MSGS;
    //memcpy(&bufI2C[0], &msgsCMD, sizeof(msgsCMD));
    I2C_send(bufI2C, sizeof(bufI2C));
    Wire.requestFrom(1, (int)dataSize);
    uint8_t dataIndex = 0;
    while(0 < Wire.available()){
      rdata[dataIndex++] = Wire.read();
    }
    
    memcpy(&msgs.wheel, &rdata[0], sizeof(msgs.wheel));
    memcpy(&msgs.encPos, &rdata[2], sizeof(msgs.encPos));
    memcpy(&msgs.RPM, &rdata[6], sizeof(msgs.RPM));
    memcpy(&msgs.PWM, &rdata[10], sizeof(msgs.PWM));
    memcpy(&msgs.refPos, &rdata[14], sizeof(msgs.refPos));
    memcpy(&msgs.refRPM, &rdata[18], sizeof(msgs.refRPM));
    memcpy(&msgs.controlFreq, &rdata[22], sizeof(msgs.controlFreq));
}
void I2C_request(float *params, uint8_t paramNum){
  uint8_t dataIndex = 0;
  Wire.requestFrom(1, 4*paramNum);
  while(0 < Wire.available()){
    rdata[dataIndex++] = Wire.read();
  }
  
  for(uint8_t pIndex = 0; pIndex < paramNum; pIndex++){
    memcpy(&params[pIndex], &rdata[pIndex*sizeof(float)], sizeof(float));
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void timersInit(){
  //Timer1 (16-bit)
  //msgs read loop Frequency: 10Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 10hz increments
  OCR1A = 24999;// = (16*10^6) / (1*64) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // 64 prescaler
  //TCCR1B |= (1 << CS11) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void enableMsgs(){
  TCCR1B |= (1 << CS11) | (1 << CS10);
}

void disableMsgs(){
  TCCR1B &= 0B11111000;
}

ISR(TIMER1_COMPA_vect){
  //Read msgs
  sei();
  int8_t msgsCMD = CMD_GET_MSGS;
  memcpy(&bufI2C[0], &msgsCMD, sizeof(msgsCMD));

  I2C_send(bufI2C, sizeof(bufI2C));
  msgsRequestParser();
}

void printMsgs(){
    Serial.print("W:"); Serial.print(msgs.wheel); Serial.print(" ");
    Serial.print("EncPos:"); Serial.print(msgs.encPos); Serial.print(" ");
    Serial.print("RPM:"); Serial.print(msgs.RPM); Serial.print(" ");
    Serial.print("PWM:"); Serial.print(msgs.PWM); Serial.print(" ");
    Serial.print("RefPos:"); Serial.print(msgs.refPos); Serial.print(" ");
    Serial.print("RefRPM:"); Serial.print(msgs.refRPM); Serial.print(" ");
    Serial.print("ControlFreq:"); Serial.print(msgs.controlFreq); Serial.print(" ");
    Serial.print("MsgsFreq:");Serial.println(msgs.msgsFreq);
}

