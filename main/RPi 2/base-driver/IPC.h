#ifndef IPC_H
#define IPC_H

#include <stdio.h>
#include <unistd.h>
#include "base.h"

extern Base mecanumBase;

//Incoming Fifo file
extern int fd;
//Outgoing Fifo file
extern int fd2;

//Motor Set CMDs
#define motor_reset             0x10
#define motor_set_RPM           0x11
#define motor_set_velocity_PID  0x12
#define motor_set_power         0x13
#define motor_set_PWM           0x14
#define motor_set_mode          0x15
//Motor Get CMDs
#define motor_get_velocity_PID  0x20
#define motor_get_pos           0x21
#define motor_get_RPM           0x22
#define motor_get_PWM           0x23
#define motor_get_ref_RPM       0x24
#define motor_get_i2c_addr      0x25

//Base Set CMDs
#define base_reset              0x30
#define base_set_velocity       0x31
#define base_set_ctrl_mode      0x32
#define base_set_velocity_PID   0x33
#define base_set_position_PID   0x34
#define base_set_goal           0x35
//Base get CMDs
#define base_get_ctrl_mode      0x40
#define base_get_velocity       0x41
#define base_get_ref_velocity   0x42
#define base_get_position       0x43
#define base_get_velocity_PID   0x44
#define base_get_position_PID   0x45
#define base_get_goal           0x46

int IPCHandler();
int IPCParser(char* IPCMSG);

#endif
