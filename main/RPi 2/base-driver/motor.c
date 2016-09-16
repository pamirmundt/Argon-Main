#include "motor.h"
#include <string.h>

//********************
//	Motor Set Commands
//********************
void motor_reset(Motor m){
	uint8_t CMD = CMD_RESET_JOINT;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
}

void motor_setRPM(Motor m, float RPM){
	uint8_t CMD = CMD_SET_JOINT_SPEED;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&i2cTXBuffer[1], &RPM, sizeof(RPM));
	i2cMasterTransmit(m.slaveAddr);
}

void motor_setPID(Motor m, float Kp, float Ki, float Kd){
	uint8_t CMD = CMD_SET_VEL_PID_CONST;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&i2cTXBuffer[1], &Kp, sizeof(Kp));
	memcpy(&i2cTXBuffer[5], &Ki, sizeof(Ki));
	memcpy(&i2cTXBuffer[9], &Kd, sizeof(Kd));
	i2cMasterTransmit(m.slaveAddr);
}

void motor_setPower(Motor m, float power){
	uint8_t CMD = CMD_SET_JOINT_POWER;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&i2cTXBuffer[1], &power, sizeof(power));
	i2cMasterTransmit(m.slaveAddr);
}

void motor_setPWM(Motor m, float PWM){
	uint8_t CMD = CMD_SET_JOINT_PWM;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&i2cTXBuffer[1], &PWM, sizeof(PWM));
	i2cMasterTransmit(m.slaveAddr);
}

void motor_setMode(Motor m, uint8_t driveMode){
	uint8_t CMD = CMD_SET_CONT_MODE;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&i2cTXBuffer[1], &driveMode, sizeof(driveMode));
	i2cMasterTransmit(m.slaveAddr);
}


//********************
//	Motor Get Commands
//********************
void motor_getPID(Motor m, float* Kp, float* Ki, float* Kd){
	uint16_t CMD = CMD_GET_VEL_PID_CONST;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	i2cMasterReceive(m.slaveAddr);
	memcpy(Kp, &i2cRXBuffer[0], sizeof(float));
	memcpy(Ki, &i2cRXBuffer[4], sizeof(float));
	memcpy(Kd, &i2cRXBuffer[8], sizeof(float));
}


float motor_getPower(Motor m){
	uint16_t CMD = CMD_GET_JOINT_POWER;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	float power;
	i2cMasterReceive(m.slaveAddr);
	memcpy(&power, &i2cRXBuffer, sizeof(power));
	return power;
}

int32_t motor_getPos(Motor m){
	uint16_t CMD = CMD_GET_JOINT_POS;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	int32_t pos;
	i2cMasterReceive(m.slaveAddr);
	memcpy(&pos, &i2cRXBuffer, sizeof(pos));
	return pos;
}

float motor_getSpeed(Motor m){
	uint16_t CMD = CMD_GET_JOINT_SPEED;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	float RPM;
	i2cMasterReceive(m.slaveAddr);
	memcpy(&RPM, &i2cRXBuffer, sizeof(RPM));
	return RPM;
}

float motor_getPWM(Motor m){
	uint16_t CMD = CMD_GET_JOINT_PWM;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	float PWM;
	i2cMasterReceive(m.slaveAddr);
	memcpy(&PWM, &i2cRXBuffer, sizeof(PWM));
	return PWM;
}

float motor_getRefSpeed(Motor m){
	uint16_t CMD = CMD_GET_JOINT_REF_SPEED;
	memcpy(&i2cTXBuffer[0], &CMD, sizeof(CMD));
	i2cMasterTransmit(m.slaveAddr);
	
	float refRPM;
	i2cMasterReceive(m.slaveAddr);
	memcpy(&refRPM, &i2cRXBuffer, sizeof(refRPM));
	return refRPM;
}

uint16_t motor_getAddr(Motor m){
	return m.slaveAddr;
}
