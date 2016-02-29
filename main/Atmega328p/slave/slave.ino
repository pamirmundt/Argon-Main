#include <digitalWriteFast.h>
#include <Wire.h>
#include "motor.h"
#include "pins.h"

//Set Motor Configurations and Functions
#define CMD_RESET 0x02
#define CMD_MOVE_SPEED 0x03
#define CMD_MOVE_TO 0x04
#define CMD_SET_PID 0x05
#define CMD_SET_POWER 0x06
#define CMD_SET_PWM 0x07
#define CMD_SET_MODE 0x08

//Get Motor Status
#define CMD_GET_PID 0x20
#define CMD_GET_POWER 0x21
#define CMD_GET_POS 0x22
#define CMD_GET_SPEED 0x23
#define CMD_GET_PWM 0x24
#define CMD_GET_REF_SPEED 0x25

#define CMD_GET_MSGS 0x30

//Control Period - 62.5Hz to microsecond  1/62.5sec = 0.016sec
#define contPerSec 0.016f
#define controlFreq 1.0/0.016f

// msgs ->    │Wheel(uint16)│EncPos(int32)│RPM(float)│PWM(float)│RefPos(int32)│RefRPM(int32)│
//                ControlFreq(float)│MsgsFreq(float)│
// bufI2C -> │CMD(uint8_t)│Param1(float)│Param2(float)│Param3(float)│

Motor m;
const uint8_t dataSize = 28;
byte bufI2C[dataSize];
byte rdata[dataSize];

//Drive Modes
typedef enum{
  MANUEL_MODE,
  POSITION_MODE,
  VELOCITY_MODE
} DRIVE_MODE;

//Default Drive Mode
DRIVE_MODE myMode = MANUEL_MODE;

//***** SETUP *****
void setup() {
  //Wheel Number
  m.wheel = 3;

  //I2C Init
  //Join I2C as Slave
  //Master Address: 0x20
  //Slave Address: 0x2X
  Wire.begin(0x24);
  Wire.onReceive(CMD_Event);
  Wire.onRequest(requestEvent);

  //Pin Init
  pinInit();
  //Set Default PID Constants
  //See "motor.h" for default values
  setDefaultKPID();
}

//***** MAIN LOOP *****
void loop() {

}

//***** I2C *****
void CMD_Event(int numBytes){
  int8_t dataIndex = 0;
  int8_t cmd = 0;
    
  while(0 < Wire.available()){
    rdata[dataIndex++] = Wire.read();
  }
  //Parse Command
  memcpy(&cmd, &rdata[0], sizeof(cmd));
  cmdParser(cmd);
}

void requestEvent(){
  Wire.write(bufI2C, sizeof(bufI2C));
}

void cmdParser(uint8_t cmd){
  switch(cmd){
    //***** SET CONFIG *****
    case CMD_RESET: {
      memset(&m, 0, sizeof(m));
      setDefaultKPID();
    } break;
    
    case CMD_MOVE_SPEED: {        //3
      //Parameters: Ref Velocity
      float params[1] = {0};
      paramParser(params, 1);
      m.refRPM = params[0];
    } break;
    
    case CMD_MOVE_TO: {
      //Parameters: Ref Position  //4
      float params[1] = {0};
      paramParser(params, 1);
      m.refPos = (int32_t)params[0];
    } break;

    case CMD_SET_PID: {           //5
      //Parameters: P, I, D
      float params[3] = {0};
      paramParser(params, 3);
      m.Kp = params[0];
      m.Ki = params[1];
      m.Kd = params[2];
    } break;

    case CMD_SET_POWER: {         //6
      //Parameters: Power
      float params[] = {0};
      paramParser(params, 1);
    } break;

    case CMD_SET_PWM: {           //7
      //Parameters: PWM - Direction
      float params[] = {0};
      paramParser(params, 1);
      m.PWM = params[0];
    } break;

    case CMD_SET_MODE:{         //8
      float params[] = {0};
      paramParser(params, 1);
      myMode = (DRIVE_MODE)params[0];
      motorInit();
    } break;
    
    //***** GET STATUS *****
    case CMD_GET_PID:   //32
      memcpy(&bufI2C[0], &m.Kp, sizeof(m.Kp));
      memcpy(&bufI2C[4], &m.Ki, sizeof(m.Ki));
      memcpy(&bufI2C[8], &m.Kd, sizeof(m.Kd));
      break;
      
    case CMD_GET_POWER: //33
    {
      float pwr = m.PWM*100.0/511.0;
      memcpy(bufI2C, &pwr, sizeof(pwr));
    } break;
    
    case CMD_GET_POS:   //34
    {
      int32_t posEnc = m.posEnc;
      memcpy(bufI2C, &posEnc, sizeof(m.posEnc));
    } break;
    
    case CMD_GET_SPEED: //35
    {
      float RPM = m.RPM/gearRatio;
      memcpy(bufI2C, &RPM, sizeof(m.RPM));
    } break;
      
    case CMD_GET_PWM:   //36
    {
      float PWM = m.PWM;
      memcpy(bufI2C, &PWM, sizeof(PWM));
    } break;
      
    case CMD_GET_REF_SPEED: //37
    {
      float refRPM = m.refRPM;
      memcpy(bufI2C, &refRPM, sizeof(m.control));
    } break;

    case CMD_GET_MSGS:  //48
    {
      // msgs - 22byte
      //│Wheel(uint16)│EncPos(int32)│RPM(float)│PWM(float)│RefPos(int32)│RefRPM(int32)│
      //      ControlFreq(float)│MsgsFreq(float)|
      int32_t posEnc = m.posEnc;
      float RPM = m.RPM/gearRatio;
      float PWM = m.PWM;
      uint32_t refPos = m.refPos;
      float refRPM = m.refRPM;
      float contFreq = controlFreq;

      memcpy(&bufI2C[0], &m.wheel, sizeof(m.wheel));
      memcpy(&bufI2C[2], &posEnc, sizeof(posEnc));
      memcpy(&bufI2C[6], &RPM, sizeof(RPM));
      memcpy(&bufI2C[10], &PWM, sizeof(PWM));
      memcpy(&bufI2C[14], &refPos, sizeof(refPos));
      memcpy(&bufI2C[18], &refRPM, sizeof(refRPM));
      memcpy(&bufI2C[22], &contFreq, sizeof(contFreq));

    } break;
    
  }
}

void paramParser(float *params, uint8_t paramNum){
  for(uint8_t pIndex = 0; pIndex < paramNum; pIndex++){
    memcpy(&params[pIndex], &rdata[pIndex*4+1], sizeof(float));
  }
}
//***** END OF I2C *****

//Control Loop
ISR(TIMER2_COMPA_vect){
  //Calculate RPM
  calcRPM();
  
  //Control Mode
  switch(myMode){
    case MANUEL_MODE: {
      m.control = m.PWM;
    } break;
    
    case POSITION_MODE: {
      calcPosPID();
    } break;
    
    case VELOCITY_MODE: {
      calcVelPID();
    } break;
  }
  
  setMotorPWM(m.control);
}

void calcVelPID(){
  //Float-PID
  m.errRPM = m.refRPM*gearRatio - m.RPM;
  m.integral = m.integral + m.errRPM*contPerSec;
  m.derivative = (m.errRPM - m.errPrevRPM)/contPerSec;
  m.control = m.Kp*m.errRPM + m.Ki*m.integral + m.Kd*m.derivative;
  m.errPrevRPM = m.errRPM;
}

void calcPosPID(){
  m.errPos = m.refPos - m.posEnc;
  m.integral = m.integral + m.errPos*contPerSec;
  m.derivative = (m.errPos - m.errPrevPos)/contPerSec;
  m.control = m.Kp*m.errPos + m.Ki*m.integral + m.Kd*m.derivative;
  m.errPrevPos = m.errPos;
}

void calcRPM(){
  m.dPos = m.posEnc - m.posEncPrev; //Delta position
  m.posEncPrev = m.posEnc;
  m.RPM = ((60.0)/(0.016*encoderRes))*m.dPos;
}

void pinInit(){
  pinMode(AIN1_pin, OUTPUT);  //AIN1 - Direction
  pinMode(AIN2_pin, OUTPUT);  //AIN2 - Direction
  pinMode(STBY_pin, OUTPUT);  //Standby - Active High
  pinMode(PWM_pin, OUTPUT);  //PWM Pin
}

void motorInit(){
  detachInterrupt(digitalPinToInterrupt(2));
  digitalWriteFast(STBY_pin, HIGH);
  timersInit();
  attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);
}

void setMotorPWM(int16_t control){
  m.PWM = constrain(control, -511, 511);
  if(m.PWM > 0){
    //CW Rotation
    digitalWriteFast(AIN1_pin, HIGH);
    digitalWriteFast(AIN2_pin, LOW);
  } else {
    //CCW Rotation
    digitalWriteFast(AIN1_pin, LOW);
    digitalWriteFast(AIN2_pin, HIGH);
  }
  analogWrite(PWM_pin, abs(m.PWM));
}

void setDefaultKPID(){
  m.Kp = Kp;
  m.Ki = Ki;
  m.Kd = Kd;
}

void timersInit(){
  //Timer1 (16-bit)
  //PWM Resolution: 512
  //PWM Frequency: 31,250Hz
  //Pins: 9-10
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)    // non-inverting PWM
       | _BV(WGM11);                    // mode 14: fast PWM, TOP=ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    // no prescaling
  ICR1 = 0x01ff;                        // TOP counter value: 511(0x1ff)

  //Timer2 (8-bit)
  //Control Loop Frequency: 62.5Hz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

//Encoder Count External Interrupt
void encoder(){
  (digitalReadFast(encoderB_pin)) ? m.posEnc++ : m.posEnc--;
}
