#ifndef I2C_H
#define I2C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

//Set Base Configuration and Function Commands
//Joint Commands
#define CMD_RESET_JOINT 				0x02
#define CMD_SET_JOINT_PWM				0x03	//Manuel mode
#define CMD_SET_JOINT_SPEED			0x04	//Velocity mode
#define CMD_SET_JOINT_POWER			0x05	//Manuel mode
//Base Commands
#define CMD_RESET_BASE					0x0A
#define CMD_BASE_START					0x0B
#define	CMD_BASE_STOP						0x0C
#define	CMD_SET_BASE_SPEED			0x0D	//Velocity mode
#define CMD_SET_BASE_GOAL				0x0E  //Position mode (need trajectory)
//Control Mode
#define CMD_SET_CONT_MODE				0x10	//Manuel/Velocity/Position Mode
//PID Constants
#define CMD_SET_VEL_PID_CONST		0x20
#define CMD_SET_POS_PID_CONST		0x21

//Get Motor Configurations and Status
//PID Constants
#define CMD_GET_VEL_PID_CONST		0x22
#define CMD_GET_POS_PID_CONST		0x23
//Joint Commands
#define CMD_GET_JOINT_PWM 					0x25
#define CMD_GET_JOINT_POS						0x26	//Encoder count
#define CMD_GET_JOINT_SPEED					0x27	//RPM
#define CMD_GET_JOINT_POWER					0x28	//0-100%
#define CMD_GET_JOINT_REF_SPEED			0x29
//Base Commands
#define CMD_GET_BASE_POS						0x2A	//x, y, theta
#define CMD_GET_BASE_SPEED					0x2B	//Vx, Vy, Vtheta
#define CMD_GET_BASE_GOAL						0x2C	//x, y, theta

/*	FUTURE
getAcceleration
getErr, getIntegral vs.
encoderREs
gearRatio
disableMotors
torqueConstant
resetEncoderCount
integralLimit
check for slaves
*/

//I2C Configurations
#define NoData         		0 // the slave has not been addressed
#define ReadAddressed  		1 // the master has requested a read from this slave (slave = transmitter)
//#define WriteGeneral   	2 // the master is writing to all slave
#define WriteAddressed 		3 // the master is writing to this slave (slave = receiver)

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

//I2C1 RX/TX Buffer
extern const uint16_t hi2c1RXBUFFERSIZE;		//I2C1 Receive buffer size
extern const uint16_t hi2c1TXBUFFERSIZE;		//I2C1 Transmit buffer size
extern uint8_t hi2c1TXBuffer[];
extern uint8_t hi2c1RXBuffer[];

//I2C2 RX/TX Buffer
extern const uint16_t hi2c2RXBUFFERSIZE;		//I2C2 Receive buffer size
extern const uint16_t hi2c2TXBUFFERSIZE;		//I2C2 Transmit buffer size
extern char hi2c2TXBuffer[];
extern char hi2c2RXBuffer[];

//********************
//	I2C Commands
//********************
void i2c1_master_transmit(uint16_t slaveAddr);
	
void i2c1_master_receive(uint16_t slaveAddr);

int hi2c2_get_state(void);

void slaveCmdParser(void);

#endif
