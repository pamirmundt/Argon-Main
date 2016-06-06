#include "motor.h"

extern UART_HandleTypeDef huart2;

//********************
//	I2C Commands
//********************
void i2c_master_transmit(uint16_t slaveAddr){
	while(HAL_I2C_Master_Transmit_DMA(&hi2c1, slaveAddr, (uint8_t*)txBuffer, TXBUFFERSIZE) != HAL_OK);		
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);		
}

void i2c_master_receive(uint16_t slaveAddr){
	while(HAL_I2C_Master_Receive_DMA(&hi2c1, slaveAddr, (uint8_t*)rxBuffer, RXBUFFERSIZE) != HAL_OK);
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
}

//********************
//	Motor Set Commands
//********************
void motor_reset(Motor m){
	uint8_t CMD = CMD_RESET;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
}

void motor_setRPM(Motor m, float RPM){
	uint8_t CMD = CMD_SET_SPEED;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	memcpy(&txBuffer[1], &RPM, sizeof(RPM));
	i2c_master_transmit(m.slaveAddr);
}

void motor_setPID(Motor m, float Kp, float Ki, float Kd){
	uint8_t CMD = CMD_SET_PID_CONST;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	memcpy(&txBuffer[1], &Kp, sizeof(Kp));
	memcpy(&txBuffer[5], &Ki, sizeof(Ki));
	memcpy(&txBuffer[9], &Kd, sizeof(Kd));
	i2c_master_transmit(m.slaveAddr);
}

void motor_setPower(Motor m, float power){
	uint8_t CMD = CMD_SET_POWER;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	memcpy(&txBuffer[1], &power, sizeof(power));
	i2c_master_transmit(m.slaveAddr);
}

void motor_setPWM(Motor m, float PWM){
	uint8_t CMD = CMD_SET_PWM;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	memcpy(&txBuffer[1], &PWM, sizeof(PWM));
	i2c_master_transmit(m.slaveAddr);
}

void motor_setMode(Motor m, uint8_t driveMode){
	uint8_t CMD = CMD_SET_MODE;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	memcpy(&txBuffer[1], &driveMode, sizeof(driveMode));
	i2c_master_transmit(m.slaveAddr);
}

//********************
//	Motor Get Commands
//********************
void motor_getPID(Motor m, float* Kp, float* Ki, float* Kd){
	uint16_t CMD = CMD_GET_PID;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	float params[3] = {0};
	i2c_master_receive(m.slaveAddr);
	memcpy(Kp, &rxBuffer[0], sizeof(float));
	memcpy(Ki, &rxBuffer[4], sizeof(float));
	memcpy(Kd, &rxBuffer[8], sizeof(float));
}

float motor_getPower(Motor m){
	uint16_t CMD = CMD_GET_POWER;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	float power;
	i2c_master_receive(m.slaveAddr);
	memcpy(&power, &rxBuffer, sizeof(power));
	return power;
}

int32_t motor_getPos(Motor m){
	uint16_t CMD = CMD_GET_POS;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	int32_t pos;
	i2c_master_receive(m.slaveAddr);
	memcpy(&pos, &rxBuffer, sizeof(pos));
	return pos;
}

float motor_getSpeed(Motor m){
	uint16_t CMD = CMD_GET_SPEED;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	float RPM;
	i2c_master_receive(m.slaveAddr);
	memcpy(&RPM, &rxBuffer, sizeof(RPM));
	return RPM;
}

float motor_getPWM(Motor m){
	uint16_t CMD = CMD_GET_PWM;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	float PWM;
	i2c_master_receive(m.slaveAddr);
	memcpy(&PWM, &rxBuffer, sizeof(PWM));
	return PWM;
}

float motor_getRefSpeed(Motor m){
	uint16_t CMD = CMD_GET_REF_SPEED;
	memcpy(&txBuffer[0], &CMD, sizeof(CMD));
	i2c_master_transmit(m.slaveAddr);
	
	float refRPM;
	i2c_master_receive(m.slaveAddr);
	memcpy(&refRPM, &rxBuffer, sizeof(refRPM));
	return refRPM;
}

uint16_t motor_getAddr(Motor m){
	return m.slaveAddr;
}
