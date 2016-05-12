#include "BaseJoint.h"

#define byte uint8_t

//Set Configuration Commands
const uint8_t CMD_RESET = 0x02;
const uint8_t CMD_MOVE_SPEED = 0x03;
const uint8_t CMD_MOVE_TO = 0x04;
const uint8_t CMD_SET_PID = 0x05;
const uint8_t CMD_SET_POWER = 0x06;
const uint8_t CMD_SET_PWM = 0x07;
const uint8_t CMD_SET_MODE = 0x08;

//Get Motor Status
const uint8_t CMD_GET_PID = 0x20;         //32
const uint8_t CMD_GET_POWER = 0x21;       //33
const uint8_t CMD_GET_POS = 0x22;         //34
const uint8_t CMD_GET_SPEED = 0x23;       //35
const uint8_t CMD_GET_PWM = 0x24;         //36
const uint8_t CMD_GET_REF_SPEED = 0x25;   //37
const uint8_t CMD_GET_REF_POS = 0x26;     //38

//Enable-Disable Status Messages(msgs)
const uint8_t CMD_GET_MSGS = 0x30;        //48

const uint8_t dataSize = 28;
byte rdata[dataSize];

byte *bufI2C;
byte *rData;

//*****I2C*****
// bufI2C ->  │CMD(uint8_t)│Param1(float)│Param2(float)│Param3(float)│

void I2C_send(byte* data, uint8_t dSize, uint8_t slaveAddr){
  Wire.beginTransmission(slaveAddr);
  Wire.write(data, dSize);
  Wire.endTransmission();
  delay(1);
}

void I2C_request(byte *params, uint8_t paramNum, uint8_t dSize, uint16_t slave){
  rData = new byte[dSize];
  Wire.requestFrom(slave, dSize);
  //delay(1);
  
  uint8_t dataIndex = 0;
  while(0 < Wire.available()){
    rData[dataIndex++] = Wire.read();
  }
  
  for(uint8_t pIndex = 0; pIndex < paramNum; pIndex++){
    memcpy(&params[pIndex*sizeof(float)], &rData[pIndex*sizeof(float)], sizeof(float));
  }
  delete[]rData;
}

//*************

//*****Wheel Class*****

//Wheel default constructor
Wheel::Wheel():slaveAddr(0),wheelNum(0), lastWheelPositionInitialized(false), lastEncPos(0){
}

//Wheel constructor
//@param wheelNumber (0-3)
//@param slaveAddress
Wheel::Wheel(uint16_t _wheelNum, uint16_t _slaveAddr){
  wheelNum = _wheelNum;
  slaveAddr = _slaveAddr;
  lastWheelPositionInitialized = false;
  lastEncPos = 0;
}

//***************
//Set Commands
//***************

//Resets the motor variables and sets PID constants to default value
void Wheel::reset(){
  bufI2C = new byte[sizeof(CMD_RESET)];
  memcpy(&bufI2C[0], &CMD_RESET, sizeof(CMD_RESET));
  I2C_send(bufI2C, sizeof(CMD_RESET), slaveAddr);
  delete[]bufI2C;
}

//Sets the Reference RPM of the wheel
//Needs to be switched to velocity control mode(2) before sending command
//@param Reference RPM
void Wheel::moveSpeed(float _RPM){
  bufI2C = new byte[sizeof(CMD_MOVE_SPEED) + sizeof(_RPM)];
  memcpy(&bufI2C[0], &CMD_MOVE_SPEED, sizeof(CMD_MOVE_SPEED));
  memcpy(&bufI2C[1], &_RPM, sizeof(_RPM));
  I2C_send(bufI2C, sizeof(CMD_MOVE_SPEED) + sizeof(_RPM), slaveAddr);
  delete[]bufI2C;
}

//Sets the Reference Position of the wheel (as encoder ticks)
//Needs to be switched to position control mode(1) before sending command
//@param Reference Position as encoder ticks
void Wheel::moveTo(int32_t _refPos){
  bufI2C = new byte[sizeof(CMD_MOVE_TO) + sizeof(_refPos)];
  memcpy(&bufI2C[0], &CMD_MOVE_TO, sizeof(CMD_MOVE_TO));
  memcpy(&bufI2C[1], &_refPos, sizeof(_refPos));
  I2C_send(bufI2C, sizeof(CMD_MOVE_TO) + sizeof(_refPos), slaveAddr);
  delete[]bufI2C;
}

//Sets the PID controller constants of the wheel
//@param proportionalConstant
//@param integralConstant
//@param derivativeConstant
void Wheel::setPID(float _Kp, float _Ki, float _Kd){
  bufI2C = new byte[sizeof(CMD_SET_PID) + sizeof(_Kp) + sizeof(_Ki) + sizeof(_Kd)];
  memcpy(&bufI2C[0], &CMD_SET_PID, sizeof(CMD_SET_PID));
  memcpy(&bufI2C[1], &_Kp, sizeof(_Kp));
  memcpy(&bufI2C[5], &_Ki, sizeof(_Ki));
  memcpy(&bufI2C[9], &_Kd, sizeof(_Kd));
  I2C_send(bufI2C, sizeof(CMD_SET_PID) + sizeof(_Kp) + sizeof(_Ki) + sizeof(_Kd), slaveAddr);
  delete[]bufI2C;
}

//Sets the power of the wheel (0-100%)
//Needs to be switched to manuel mode(0) before sending command
//@param power
void Wheel::setPower(float _power){
  bufI2C = new byte[sizeof(CMD_SET_POWER) + sizeof(_power)];
  memcpy(&bufI2C[0], &CMD_SET_POWER, sizeof(CMD_SET_POWER));
  memcpy(&bufI2C[1], &_power, sizeof(_power));
  I2C_send(bufI2C, sizeof(CMD_SET_POWER) + sizeof(_power), slaveAddr);
  delete[]bufI2C;
}

//Sets the PWM of the wheel (9bit resolution, 0-511)
//Needs to be switched to manuel mode(0) before sending command
//@param PWM
void Wheel::setPWM(float _PWM){
  bufI2C = new byte[sizeof(CMD_SET_PWM) + sizeof(_PWM)];
  memcpy(&bufI2C[0], &CMD_SET_PWM, sizeof(CMD_SET_PWM));
  memcpy(&bufI2C[1], &_PWM, sizeof(_PWM));
  I2C_send(bufI2C, sizeof(CMD_SET_PWM) + sizeof(_PWM), slaveAddr);
  delete[]bufI2C;
}

//Sets the control mode of the wheel
//0 - Manuel Mode (setPower, setPWM, etc.)
//1 - Position Control Mode with PID
//2 - Velocity Control Mode with PID
//@param driveMode
void Wheel::setMode(uint8_t _driveMode){
  bufI2C = new byte[sizeof(CMD_SET_MODE) + sizeof(_driveMode)];
  memcpy(&bufI2C[0], &CMD_SET_MODE, sizeof(CMD_SET_MODE));
  memcpy(&bufI2C[1], &_driveMode, sizeof(_driveMode));
  I2C_send(bufI2C, sizeof(CMD_SET_MODE) + sizeof(_driveMode), slaveAddr);
  delete[]bufI2C;
}


//***************
//Get Commands
//***************

//Get the current PID constrants of the wheel
//@param proportionalConstant
//@param integralConstant
//@param derivativeConstant
void Wheel::getPID(float* _Kp, float* _Ki, float* _Kd){
  
  bufI2C = new byte[sizeof(CMD_GET_PID)];
  memcpy(&bufI2C[0], &CMD_GET_PID, sizeof(CMD_GET_PID));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(float)*3] = {};
  uint8_t aLength = sizeof(params)/sizeof(float);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  memcpy(_Kp, &params[0], sizeof(float));
  memcpy(_Ki, &params[4], sizeof(float));
  memcpy(_Kd, &params[8], sizeof(float));
}

//Returns the current power (0-100%)
float Wheel::getPower(){
  bufI2C = new byte[sizeof(CMD_GET_POWER)];
  memcpy(&bufI2C[0], &CMD_GET_POWER, sizeof(CMD_GET_POWER));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(float)] = {};
  uint8_t aLength = sizeof(params)/sizeof(float);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  float _pwr;
  memcpy(&_pwr, &params[0], sizeof(float));
  return _pwr;
}

//Returns the position as encoder Ticks
int32_t Wheel::getPos(){
  bufI2C = new byte[sizeof(CMD_GET_POS)];
  memcpy(&bufI2C[0], &CMD_GET_POS, sizeof(CMD_GET_POS));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(int32_t)] = {};
  uint8_t aLength = sizeof(params)/sizeof(int32_t);
  I2C_request(params, aLength, sizeof(params), slaveAddr);
  
  int32_t _encPos = 0;
  memcpy(&_encPos, &params[0], sizeof(int32_t));
  return _encPos;
}

//Returns the velocity as RPM
float Wheel::getSpeed(){
  bufI2C = new byte[sizeof(CMD_GET_SPEED)];
  memcpy(&bufI2C[0], &CMD_GET_SPEED, sizeof(CMD_GET_SPEED));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(float)] = {};
  uint8_t aLength = sizeof(params)/sizeof(float);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  float _RPM;
  memcpy(&_RPM, &params[0], sizeof(float));
  return _RPM;
}

//Returns the PWM (0-511)
float Wheel::getPWM(){
  bufI2C = new byte[sizeof(CMD_GET_PWM)];
  memcpy(&bufI2C[0], &CMD_GET_PWM, sizeof(CMD_GET_PWM));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(float)] = {};
  uint8_t aLength = sizeof(params)/sizeof(float);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  float _PWM;
  memcpy(&_PWM, &params[0], sizeof(float));
  return _PWM;
}

//Returns the reference speed as RPM
float Wheel::getRefSpeed(){
  bufI2C = new byte[sizeof(CMD_GET_REF_SPEED)];
  memcpy(&bufI2C[0], &CMD_GET_REF_SPEED, sizeof(CMD_GET_REF_SPEED));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(float)] = {};
  uint8_t aLength = sizeof(params)/sizeof(float);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  float _refSpeed;
  memcpy(&_refSpeed, &params[0], sizeof(float));
  return _refSpeed;
}

//Returns the I2C address
int32_t Wheel::getRefPos(){
  bufI2C = new byte[sizeof(CMD_GET_REF_POS)];
  memcpy(&bufI2C[0], &CMD_GET_REF_POS, sizeof(CMD_GET_REF_POS));
  I2C_send(bufI2C, sizeof(bufI2C), slaveAddr);
  delete[]bufI2C;

  byte params[sizeof(int32_t)] = {};
  uint8_t aLength = sizeof(params)/sizeof(int32_t);
  I2C_request(params, aLength, sizeof(params), slaveAddr);

  int32_t _refSpeed;
  memcpy(&_refSpeed, &params[0], sizeof(int32_t));
  return _refSpeed;
}

//Returns the I2C address
uint16_t Wheel::getAddr(){
  return slaveAddr;
}

//Returns the number of the wheel (0-3)
uint16_t Wheel::getWheelNum(){
  return wheelNum;
}