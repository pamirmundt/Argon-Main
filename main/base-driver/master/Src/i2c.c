#include "i2c.h"
#include "base.h"

//Base
extern Base mecanumBase;

//I2C1 RX/TX Buffer
const uint16_t hi2c1RXBUFFERSIZE = 12;		//I2C1 Receive buffer size
const uint16_t hi2c1TXBUFFERSIZE = 13;		//I2C1 Transmit buffer size
uint8_t hi2c1TXBuffer[hi2c1TXBUFFERSIZE];
uint8_t hi2c1RXBuffer[hi2c1RXBUFFERSIZE];

//I2C2 RX/TX Buffer
const uint16_t hi2c2RXBUFFERSIZE = 17;		//I2C2 Receive buffer size
const uint16_t hi2c2TXBUFFERSIZE = 16;		//I2C2 Transmit buffer size
char hi2c2TXBuffer[hi2c2TXBUFFERSIZE];
char hi2c2RXBuffer[hi2c2RXBUFFERSIZE];

//DEBUG
extern UART_HandleTypeDef huart2;

//********************
//	I2C Commands
//********************
void i2c1_master_transmit(uint16_t slaveAddr){
	while(HAL_I2C_Master_Transmit_DMA(&hi2c1, slaveAddr, (uint8_t*)hi2c1TXBuffer, hi2c1TXBUFFERSIZE) != HAL_OK);		
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);		
}

void i2c1_master_receive(uint16_t slaveAddr){
	while(HAL_I2C_Master_Receive_DMA(&hi2c1, slaveAddr, (uint8_t*)hi2c1RXBuffer, hi2c1RXBUFFERSIZE) != HAL_OK);
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
}

//I2C2 get state
int hi2c2_get_state(){
	int retValue = NoData;

    if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) == 1) {
        if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_ADDR) == 1) {
            if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_TRA) == 1)
                retValue = ReadAddressed;
            else
                retValue = WriteAddressed;

            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ADDR);
        }
	}
		
	return (retValue);
}

//f446re as Slave - I2C2
void slaveCmdParser(){
	//Clean TX Buffer before writing new messages
	memset(&hi2c2TXBuffer, 0, sizeof(hi2c2TXBUFFERSIZE));
	
	//Get Command - First byte of RXBUFFER
	uint8_t cmd;
	memcpy(&cmd, &hi2c2RXBuffer[0], sizeof(cmd));
	
	//DEBUG
	//char str[30];
	//sprintf(str,"%d %d \n", cmd, test);
	//HAL_UART_Transmit_IT(&huart2,(uint8_t*)str, strlen(str));
	
	switch(cmd){
		//*************************
		//	Joint Set Commands
		//*************************
		case CMD_RESET_JOINT:{
			motor_reset(mecanumBase.frontLeftWheel);
			motor_reset(mecanumBase.frontRightWheel);
			motor_reset(mecanumBase.rearLeftWheel);
			motor_reset(mecanumBase.rearRightWheel);
		} break;
		
		case CMD_SET_JOINT_PWM:{
			float params[4] = {0};
			memcpy(&params[0], &hi2c2RXBuffer[1], sizeof(float));
			memcpy(&params[1], &hi2c2RXBuffer[5], sizeof(float));
			memcpy(&params[2], &hi2c2RXBuffer[9], sizeof(float));
			memcpy(&params[3], &hi2c2RXBuffer[13], sizeof(float));
			
			motor_setPWM(mecanumBase.frontLeftWheel, params[0]);
			motor_setPWM(mecanumBase.frontRightWheel, params[1]);
			motor_setPWM(mecanumBase.rearLeftWheel, params[2]);
			motor_setPWM(mecanumBase.rearRightWheel, params[3]);
		} break;
		
		case CMD_SET_JOINT_SPEED:{
			float params[4] = {0};
			memcpy(&params[0], &hi2c2RXBuffer[1], sizeof(float));
			memcpy(&params[1], &hi2c2RXBuffer[5], sizeof(float));
			memcpy(&params[2], &hi2c2RXBuffer[9], sizeof(float));
			memcpy(&params[3], &hi2c2RXBuffer[13], sizeof(float));
			
			motor_setRPM(mecanumBase.frontLeftWheel, params[0]);
			motor_setRPM(mecanumBase.frontRightWheel, params[1]);
			motor_setRPM(mecanumBase.rearLeftWheel, params[2]);
			motor_setRPM(mecanumBase.rearRightWheel, params[3]);
		} break;
		
		case CMD_SET_JOINT_POWER:{
			float params[4] = {0};
			memcpy(&params[0], &hi2c2RXBuffer[1], sizeof(float));
			memcpy(&params[1], &hi2c2RXBuffer[5], sizeof(float));
			memcpy(&params[2], &hi2c2RXBuffer[9], sizeof(float));
			memcpy(&params[3], &hi2c2RXBuffer[13], sizeof(float));
			
			motor_setPower(mecanumBase.frontLeftWheel, params[0]);
			motor_setPower(mecanumBase.frontRightWheel, params[1]);
			motor_setPower(mecanumBase.rearLeftWheel, params[2]);
			motor_setPower(mecanumBase.rearRightWheel, params[3]);
		} break;
		
		//*************************
		//	Base Set Commands
		//*************************
		case CMD_RESET_BASE:{
			motor_reset(mecanumBase.frontLeftWheel);
			motor_reset(mecanumBase.frontRightWheel);
			motor_reset(mecanumBase.rearLeftWheel);
			motor_reset(mecanumBase.rearRightWheel);
			base_reset(&mecanumBase);		
		} break;
		
		case CMD_BASE_START:{
			
		} break;
		
		case CMD_BASE_STOP:{
			
		} break;
		
		case CMD_SET_BASE_SPEED:{
			float params[3] = {0};
			memcpy(&params[0], &hi2c2RXBuffer[1], sizeof(float));		//Longitudinal Velocity
			memcpy(&params[1], &hi2c2RXBuffer[5], sizeof(float));		//Transversal Velocity
			memcpy(&params[2], &hi2c2RXBuffer[9], sizeof(float));		//Angular Velocity
			
			base_setVelocity(mecanumBase, params[0], params[1], params[2]);
		} break;
		
		case CMD_SET_BASE_GOAL:{
			
		} break;
		
		//Control Mode
		case CMD_SET_CONT_MODE:{
			uint8_t params[1] = {0};
			memcpy(&params[0], &hi2c2RXBuffer[1], sizeof(uint8_t));
			
			base_setControlMode(&mecanumBase, params[0]);
		} break;
		
		//PID
		case CMD_SET_VEL_PID_CONST:{
			
		} break;
		
		case CMD_SET_POS_PID_CONST:{
			
		} break;
		
		//****************************
		//	Get Commands
		//****************************
		//Joint Commands
		case CMD_GET_JOINT_PWM:{
			float params[4] = {0};
			params[0] = motor_getPWM(mecanumBase.frontLeftWheel);
			params[1] = motor_getPWM(mecanumBase.frontRightWheel);
			params[2] = motor_getPWM(mecanumBase.rearLeftWheel);
			params[3] = motor_getPWM(mecanumBase.rearRightWheel);
			
			memcpy(&hi2c2TXBuffer[0], &params[0], sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &params[1], sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &params[2], sizeof(float));
			memcpy(&hi2c2TXBuffer[12], &params[3], sizeof(float));
			
		} break;
		
		case CMD_GET_JOINT_POS:{
			int32_t params[4] = {0};
			params[0] = motor_getPos(mecanumBase.frontLeftWheel);
			params[1] = motor_getPos(mecanumBase.frontRightWheel);
			params[2] = motor_getPos(mecanumBase.rearLeftWheel);
			params[3] = motor_getPos(mecanumBase.rearRightWheel);
			
			memcpy(&hi2c2TXBuffer[0], &params[0], sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &params[1], sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &params[2], sizeof(float));
			memcpy(&hi2c2TXBuffer[12], &params[3], sizeof(float));
		} break;
		
		case CMD_GET_JOINT_SPEED:{
			float params[4] = {0};
			params[0] = motor_getSpeed(mecanumBase.frontLeftWheel);
			params[1] = motor_getSpeed(mecanumBase.frontRightWheel);
			params[2] = motor_getSpeed(mecanumBase.rearLeftWheel);
			params[3] = motor_getSpeed(mecanumBase.rearRightWheel);
			
			memcpy(&hi2c2TXBuffer[0], &params[0], sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &params[1], sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &params[2], sizeof(float));
			memcpy(&hi2c2TXBuffer[12], &params[3], sizeof(float));
		} break;
	
		case CMD_GET_JOINT_POWER:{
			float params[4] = {0};
			params[0] = motor_getPower(mecanumBase.frontLeftWheel);
			params[1] = motor_getPower(mecanumBase.frontRightWheel);
			params[2] = motor_getPower(mecanumBase.rearLeftWheel);
			params[3] = motor_getPower(mecanumBase.rearRightWheel);
			
			memcpy(&hi2c2TXBuffer[0], &params[0], sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &params[1], sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &params[2], sizeof(float));
			memcpy(&hi2c2TXBuffer[12], &params[3], sizeof(float));
		} break;

		case CMD_GET_JOINT_REF_SPEED:{

		} break;
		
		//Base Commands
		case CMD_GET_BASE_POS:{
			float positions[3] = {0};
			base_getPosition(&mecanumBase, positions);
			
			memcpy(&hi2c2TXBuffer[0], &positions[0], sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &positions[1], sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &positions[2], sizeof(float));
		} break;
		
		case CMD_GET_BASE_SPEED:{
			float longitudinalVel = 0.0f, transversalVel = 0.0f, angularVel = 0.0f;
			base_getVelocity(mecanumBase, &longitudinalVel, &transversalVel, &angularVel);
			
			memcpy(&hi2c2TXBuffer[0], &longitudinalVel, sizeof(float));
			memcpy(&hi2c2TXBuffer[4], &transversalVel, sizeof(float));
			memcpy(&hi2c2TXBuffer[8], &angularVel, sizeof(float));
		} break;
		
		case CMD_GET_BASE_GOAL:{
			
		} break;
		
		case CMD_GET_VEL_PID_CONST:{
			
		} break;
		
		case CMD_GET_POS_PID_CONST:{
			
		} break;
	}
	//Clear RX Buffer before receiving new data
	memset(&hi2c2RXBuffer, 0, sizeof(hi2c2RXBUFFERSIZE));
}
