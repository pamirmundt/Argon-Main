//*****************************
//* Motor I2C Communication
//*****************************

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "i2c.h"

/*
//Set Motor Configuration and Function Commands
#define CMD_RESET 				0x02
#define CMD_SET_SPEED			0x03	//Velocity control mode Only
#define CMD_SET_PID_CONST	0x04
#define CMD_SET_POWER			0x05	//Manuel mode only
#define CMD_SET_PWM				0x06	//Manuel mode only
#define CMD_SET_MODE			0x07

//Get Motor Configurations and Status
#define CMD_GET_PID 			0x10
#define CMD_GET_POWER 		0x11
#define CMD_GET_POS 			0x12
#define CMD_GET_SPEED 		0x13
#define CMD_GET_PWM 			0x14
#define CMD_GET_REF_SPEED 0x15
*/

typedef struct{
	uint16_t slaveAddr;
	int32_t lastEncPos;
}Motor;


//********************
//	Set Commands
//********************

//Resets the motor variables and sets PID constants to default value
//@param Motor
void motor_reset(Motor m);

//Sets the Reference RPM of the wheel
//Needs to be switched to velocity control mode(2) before sending command
//@param Motor
//@param Reference RPM 
void motor_setRPM(Motor m, float RPM);

//Sets the PID controller constants of the wheel
//@param Motor
//@param proportionalConstant
//@param integralConstant
//@param derivativeConstant
void motor_setPID(Motor m, float Kp, float Ki, float Kd);

//Sets the power of the wheel (0-100%)
//Needs to be switched to manuel mode(0) before sending command
//@param Motor
//@param power
void motor_setPower(Motor m, float power);

//Sets the PWM of the wheel (9bit resolution, 0-511)
//Needs to be switched to manuel mode(0) before sending command
//@param Motor
//@param PWM
void motor_setPWM(Motor m, float PWM);

//Sets the control mode of the wheel
//0 - Manuel Mode (setPower, setPWM, etc.)
//1 - Position Control Mode with PID
//2 - Velocity Control Mode with PID
//@param motor
//@param driveMode
void motor_setMode(Motor m, uint8_t driveMode);

//********************
//	Get Commands
//********************

//Get the current PID constrants of the wheel
//@param motor
//@param proportionalConstant
//@param integralConstant
//@param derivativeConstant
void motor_getPID(Motor m, float* Kp, float* Ki, float* Kd);

//Returns the current power (-100% +100%)
//@param Motor
//@retval power(-100% +100%) (float)
float motor_getPower(Motor m);

//Returns the position as encoder Ticks
//@param Motor
//@retval Encoder position (int32_t)
int32_t motor_getPos(Motor m);

//Returns the velocity as RPM
//@param Motor
//@retval Wheel RPM (float)
float motor_getSpeed(Motor m);

//Returns the PWM - 12Bit (0-4098)
//@param Motor
//@retval PWM (float)
float motor_getPWM(Motor m);

//Returns the reference speed as RPM
//@param Motor
//@retval Reference RPM (float)
float motor_getRefSpeed(Motor m);

//Returns the I2C address
//@param Motor
//@retval Motor I2C slave address (uint16_t)
uint16_t motor_getAddr(Motor m);

#endif
