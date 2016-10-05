#ifndef IPC_H
#define IPC_H

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include "base.h"

extern Base mecanumBase;

//Incoming Fifo file
extern int fd;
//Outgoing Fifo file
extern int fd2;

//Motor Set CMDs
#define CMD_MOTOR_RESET             0x10
#define CMD_MOTOR_SET_RPM           0x11
#define CMD_MOTOR_SET_VELOCITY_PID  0x12
#define CMD_MOTOR_SET_POWER         0x13
#define CMD_MOTOR_SET_PWM           0x14
#define CMD_MOTOR_SET_MODE          0x15
//Motor Get CMDs
#define CMD_MOTOR_GET_VELOCITY_PID  0x20
#define CMD_MOTOR_GET_POS           0x21
#define CMD_MOTOR_GET_RPM           0x22
#define CMD_MOTOR_GET_PWM           0x23
#define CMD_MOTOR_GET_REF_RPM       0x24
#define CMD_MOTOR_GET_I2C_ADDR      0x25

//Base Set CMDs
#define CMD_BASE_RESET          	0x30
#define CMD_BASE_SET_VELOCITY       0x31
#define CMD_BASE_SET_CTRL_MODE      0x32
#define CMD_BASE_SET_VELOCITY_PID   0x33
#define CMD_BASE_SET_POSITION_PID   0x34 	//NYI
#define CMD_BASE_SET_GOAL           0x35	//NYI
//Base get CMDs
#define CMD_BASE_GET_CTRL_MODE      0x40
#define CMD_BASE_GET_VELOCITY       0x41
#define CMD_BASE_GET_REF_VELOCITY   0x42
#define CMD_BASE_GET_POSITION       0x43
#define CMD_BASE_GET_VELOCITY_PID   0x44
#define CMD_BASE_GET_POSITION_PID   0x45	//NYI
#define CMD_BASE_GET_GOAL           0x46	//NYI

int fifo_read(char rx[], uint8_t size);
int fifo_write(char tx[], uint8_t size);
int IPCHandler();
Motor motorSelect(uint8_t wheelNumber);
int IPCParser(char* IPCMSG);

#endif
