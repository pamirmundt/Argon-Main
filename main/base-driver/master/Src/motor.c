#include "motor.h"

//********************
//	Motor Set Commands
//********************
void motor_reset(Motor m){
	uint8_t CMD = CMD_RESET_JOINT;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
}

void motor_setRPM(Motor m, float RPM){
	uint8_t CMD = CMD_SET_JOINT_SPEED;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&hi2c1TXBuffer[1], &RPM, sizeof(RPM));
	i2c1_master_transmit(m.slaveAddr);
}

void motor_setPID(Motor m, float Kp, float Ki, float Kd){
	uint8_t CMD = CMD_SET_VEL_PID_CONST;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&hi2c1TXBuffer[1], &Kp, sizeof(Kp));
	memcpy(&hi2c1TXBuffer[5], &Ki, sizeof(Ki));
	memcpy(&hi2c1TXBuffer[9], &Kd, sizeof(Kd));
	i2c1_master_transmit(m.slaveAddr);
}

void motor_setPower(Motor m, float power){
	uint8_t CMD = CMD_SET_JOINT_POWER;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&hi2c1TXBuffer[1], &power, sizeof(power));
	i2c1_master_transmit(m.slaveAddr);
}

void motor_setPWM(Motor m, float PWM){
	uint8_t CMD = CMD_SET_JOINT_PWM;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&hi2c1TXBuffer[1], &PWM, sizeof(PWM));
	i2c1_master_transmit(m.slaveAddr);
}

void motor_setMode(Motor m, uint8_t driveMode){
	uint8_t CMD = CMD_SET_CONT_MODE;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	memcpy(&hi2c1TXBuffer[1], &driveMode, sizeof(driveMode));
	i2c1_master_transmit(m.slaveAddr);
}

//********************
//	Motor Get Commands
//********************
void motor_getPID(Motor m, float* Kp, float* Ki, float* Kd){
	uint16_t CMD = CMD_GET_VEL_PID_CONST;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	float params[3] = {0};
	i2c1_master_receive(m.slaveAddr);
	memcpy(Kp, &hi2c1RXBuffer[0], sizeof(float));
	memcpy(Ki, &hi2c1RXBuffer[4], sizeof(float));
	memcpy(Kd, &hi2c1RXBuffer[8], sizeof(float));
}

float motor_getPower(Motor m){
	uint16_t CMD = CMD_GET_JOINT_POWER;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	float power;
	i2c1_master_receive(m.slaveAddr);
	memcpy(&power, &hi2c1RXBuffer, sizeof(power));
	return power;
}

int32_t motor_getPos(Motor m){
	uint16_t CMD = CMD_GET_JOINT_POS;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	int32_t pos;
	i2c1_master_receive(m.slaveAddr);
	memcpy(&pos, &hi2c1RXBuffer, sizeof(pos));
	return pos;
}

float motor_getSpeed(Motor m){
	uint16_t CMD = CMD_GET_JOINT_SPEED;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	float RPM;
	i2c1_master_receive(m.slaveAddr);
	memcpy(&RPM, &hi2c1RXBuffer, sizeof(RPM));
	return RPM;
}

float motor_getPWM(Motor m){
	uint16_t CMD = CMD_GET_JOINT_PWM;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	float PWM;
	i2c1_master_receive(m.slaveAddr);
	memcpy(&PWM, &hi2c1RXBuffer, sizeof(PWM));
	return PWM;
}

float motor_getRefSpeed(Motor m){
	uint16_t CMD = CMD_GET_JOINT_REF_SPEED;
	memcpy(&hi2c1TXBuffer[0], &CMD, sizeof(CMD));
	i2c1_master_transmit(m.slaveAddr);
	
	float refRPM;
	i2c1_master_receive(m.slaveAddr);
	memcpy(&refRPM, &hi2c1RXBuffer, sizeof(refRPM));
	return refRPM;
}

uint16_t motor_getAddr(Motor m){
	return m.slaveAddr;
}
