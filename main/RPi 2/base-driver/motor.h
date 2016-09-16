//*****************************
// Motor RPi I2C Communication
//*****************************

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "RPi-i2c.h"

//Set Base Configuration and Function Commands
//Joint Commands
#define CMD_RESET_JOINT 				0x02
#define CMD_SET_JOINT_PWM				0x03	//Manuel mode
#define CMD_SET_JOINT_SPEED				0x04	//Velocity mode
#define CMD_SET_JOINT_POWER				0x05	//Manuel mode

//Control Mode
#define CMD_SET_CONT_MODE				0x10	//Manuel/Velocity

//PID
#define CMD_SET_VEL_PID_CONST			0x20
#define CMD_GET_VEL_PID_CONST			0x22

//Get Motor Configurations and Status
//Joint Commands
#define CMD_GET_JOINT_PWM 				0x25
#define CMD_GET_JOINT_POS				0x26	//Encoder count
#define CMD_GET_JOINT_SPEED				0x27	//RPM
#define CMD_GET_JOINT_POWER				0x28	//0-100%
#define CMD_GET_JOINT_REF_SPEED			0x29


typedef struct{
	uint16_t slaveAddr;

	int32_t encoderPosition;
	int32_t RPM;
	//int32_t lastEncPos;
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