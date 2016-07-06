#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <Python.h>
#include "i2c-dev.h"

//Set Base Configuration and Function Commands
//Joint Commands
#define CMD_RESET_JOINT 				0x02
#define CMD_SET_JOINT_PWM				0x03
#define CMD_SET_JOINT_SPEED				0x04
#define CMD_SET_JOINT_POWER				0x05
//Base Commands
#define CMD_RESET_BASE					0x0A
#define CMD_BASE_START					0x0B	//NOT YET IMPLEMENTED
#define	CMD_BASE_STOP					0x0C 	//NOT YET IMPLEMENTED
#define	CMD_SET_BASE_SPEED				0x0D
#define CMD_SET_BASE_GOAL				0x0E
//Control Mode
#define CMD_SET_CONT_MODE				0x10
//PID
#define CMD_SET_VEL_PID_CONST				0x20
#define CMD_SET_POS_PID_CONST				0x21
#define CMD_GET_VEL_PID_CONST				0x22
#define CMD_GET_POS_PID_CONST				0x23

//Get Motor Configurations and Status
//Joint Commands
#define CMD_GET_JOINT_PWM 				0x25
#define CMD_GET_JOINT_POS				0x26
#define CMD_GET_JOINT_SPEED				0x27
#define CMD_GET_JOINT_POWER				0x28
#define CMD_GET_JOINT_REF_SPEED				0x29
//Base Commands
#define CMD_GET_BASE_POS				0x2A
#define CMD_GET_BASE_SPEED				0x2B
#define CMD_GET_BASE_GOAL				0x2C	//NYI


//I2C TX/RX Buffers
#define RXBUFFERSIZE 16
#define TXBUFFERSIZE 17

uint8_t rxBuffer[RXBUFFERSIZE];
uint8_t txBuffer[TXBUFFERSIZE];

//Function Prototypes
int init_I2C(void);
int open_Port(int devId);
inline int masterWriteBlock(int file, uint8_t size, uint8_t buff[]);
inline int masterReadBlock(int file, uint8_t size, uint8_t buff[]);


int joint_reset(void);
int joint_setPWM(float fl_pwm, float fr_pwm, float rl_pwm, float rr_pwm);
int joint_setSpeed(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm);
int joint_setPower(float fl_pwr, float fr_pwr, float rl_pwr, float rr_pwr);

int base_reset(void);
int base_setSpeed(float longVel, float tranVel, float angVel);
int set_VelPIDConst(uint8_t motorNum, float PConst, float IConst, float DConst);
int set_PosPIDConst(uint8_t mode, float PConst, float IConst, float DConst);
int set_ControlMode(uint8_t mode);

int get_VelPIDConst(uint8_t motorNum, float PIDConsts[]);
int get_PosPIDConst(uint8_t mode, float PIDConsts[]);
int joint_getPos(int32_t jointPos[]);
int joint_getPWM(float jointPWM[]);
int joint_getSpeed(float jointSpeed[]);
int joint_getPower(float jointPower[]);
int joint_getRefSpeed(float jointRefSpeed[]);
int base_getPos(float basePos[]);
int base_getSpeed(float baseSpeed[]);

//Variables
static int fd;


/********************************************************************
	Python extend with C for Raspberry Pi 2
	Link: https://docs.python.org/2/extending/extending.html
********************************************************************/

/***************
	SMBus I2C
***************/

static PyObject* initI2C(PyObject *self, PyObject *args)
{
	int err;
	err = init_I2C();
	return Py_BuildValue("i", err);
}

static PyObject* openPort(PyObject *self, PyObject *args)
{
	int err;
	int slaveAddr;
	if (!PyArg_ParseTuple(args, "i", &slaveAddr))
        	return NULL;

	err = open_Port(slaveAddr);
		
	return Py_BuildValue("i", err);
}

/****************
	Mecanum Base
*****************/

static PyObject* jointReset(PyObject *self, PyObject *args)
{
	int err;
	err = joint_reset();

	return Py_BuildValue("i", err);
}

static PyObject* jointSetPWM(PyObject *self, PyObject *args)
{
	int err;
	float jointPWM[4] = {0};
	if (!PyArg_ParseTuple(args, "ffff", &jointPWM[0], &jointPWM[1], &jointPWM[2], &jointPWM[3]))
        	return NULL;

	err = joint_setPWM(jointPWM[0], jointPWM[1], jointPWM[2], jointPWM[3]);
			
	return Py_BuildValue("i", err);
}

static PyObject* jointSetSpeed(PyObject *self, PyObject *args)
{
	int err;
	float jointSpeed[4] = {0};
	if (!PyArg_ParseTuple(args, "ffff", &jointSpeed[0], &jointSpeed[1], &jointSpeed[2], &jointSpeed[3]))
        	return NULL;

	err = joint_setSpeed(jointSpeed[0], jointSpeed[1], jointSpeed[2], jointSpeed[3]);
			
	return Py_BuildValue("i", err);
}

static PyObject* jointSetPower(PyObject *self, PyObject *args)
{
	int err;
	float jointPower[4] = {0};
	if (!PyArg_ParseTuple(args, "ffff", &jointPower[0], &jointPower[1], &jointPower[2], &jointPower[3]))
        	return NULL;

	err = joint_setPower(jointPower[0], jointPower[1], jointPower[2], jointPower[3]);
			
	return Py_BuildValue("i", err);
}

static PyObject* baseReset(PyObject *self, PyObject *args)
{
	int err;

	err = base_reset();
			
	return Py_BuildValue("i", err);
}

static PyObject* baseSetSpeed(PyObject *self, PyObject *args)
{
	int err;
	float speed[3] = {0};
	if (!PyArg_ParseTuple(args, "fff", &speed[0], &speed[1], &speed[2]))
        	return NULL;

	err = base_setSpeed(speed[0], speed[1], speed[2]);
			
	return Py_BuildValue("i", err);
}

static PyObject* setVelPIDConst(PyObject *self, PyObject *args)
{
	int err;
	uint8_t motorNum;
	float PIDConsts[3] = {0};
	if (!PyArg_ParseTuple(args, "ifff", &motorNum, &PIDConsts[0], &PIDConsts[1], &PIDConsts[2]))
        	return NULL;

	err = set_VelPIDConst(motorNum, PIDConsts[0], PIDConsts[1], PIDConsts[2]);
			
	return Py_BuildValue("i", err);
}

static PyObject* setPosPIDConst(PyObject *self, PyObject *args)
{
	int err;
	uint8_t mode;
	float PIDConsts[3] = {0};
	if (!PyArg_ParseTuple(args, "ifff", &mode, &PIDConsts[0], &PIDConsts[1], &PIDConsts[2]))
        	return NULL;

	err = set_PosPIDConst(mode, PIDConsts[0], PIDConsts[1], PIDConsts[2]);
			
	return Py_BuildValue("i", err);
}

static PyObject* setControlMode(PyObject *self, PyObject *args)
{
	int err;
	uint8_t mode;

	if (!PyArg_ParseTuple(args, "b", &mode))
        	return NULL;	

	err = set_ControlMode(mode);

	return Py_BuildValue("i", err);
}

static PyObject* getVelPIDConst(PyObject *self, PyObject *args)
{
	float PIDConsts[3] = {};
	uint8_t motorNum;

	//MotorNum:0 - Front Left Motor
	//MotorNum:1 - Front Right Motor
	//MotorNum:2 - Rear Left Motor
	//MotorNum:3 - Rear Right Motor
	if (!PyArg_ParseTuple(args, "i", &motorNum))
        	return NULL;	

	PyObject * pylist = PyList_New(3);

	if(get_VelPIDConst(motorNum, PIDConsts) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 3; i++){
		PyObject * item = Py_BuildValue("f", PIDConsts[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* getPosPIDConst(PyObject *self, PyObject *args)
{
	float PIDConsts[3] = {};
	uint8_t mode;

	//Mode:0 - Longitudal PID Constants
	//Mode:1 - Transversal PID Constants
	//Mode:2 - Angular PID Constants
	if (!PyArg_ParseTuple(args, "i", &mode))
        	return NULL;	

	PyObject * pylist = PyList_New(3);

	if(get_PosPIDConst(mode, PIDConsts) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 3; i++){
		PyObject * item = Py_BuildValue("f", PIDConsts[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* jointGetPos(PyObject *self, PyObject *args)
{
	int32_t jointPos[4] = {};

	PyObject * pylist = PyList_New(4);

	if(joint_getPos(jointPos) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 4; i++){
		PyObject * item = Py_BuildValue("i", jointPos[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* jointGetPWM(PyObject *self, PyObject *args)
{
	float jointPWM[4] = {};

	PyObject * pylist = PyList_New(4);

	if(joint_getPWM(jointPWM) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 4; i++){
		PyObject * item = Py_BuildValue("f", jointPWM[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}


static PyObject* jointGetSpeed(PyObject *self, PyObject *args)
{
	float jointSpeed[4] = {};

	PyObject * pylist = PyList_New(4);

	if(joint_getSpeed(jointSpeed) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 4; i++){
		PyObject * item = Py_BuildValue("f", jointSpeed[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* jointGetPower(PyObject *self, PyObject *args)
{
	float jointPower[4] = {};

	PyObject * pylist = PyList_New(4);

	if(joint_getPower(jointPower) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 4; i++){
		PyObject * item = Py_BuildValue("f", jointPower[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* jointGetRefSpeed(PyObject *self, PyObject *args)
{
	float jointPower[4] = {};

	PyObject * pylist = PyList_New(4);

	if(joint_getRefSpeed(jointPower) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 4; i++){
		PyObject * item = Py_BuildValue("f", jointPower[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* baseGetPos(PyObject *self, PyObject *args)
{
	float basePosition[3] = {};

	PyObject * pylist = PyList_New(3);

	if(base_getPos(basePosition) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 3; i++){
		PyObject * item = Py_BuildValue("f", basePosition[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyObject* baseGetSpeed(PyObject *self, PyObject *args)
{
	float baseSpeed[3] = {};

	PyObject * pylist = PyList_New(3);

	if(base_getSpeed(baseSpeed) == -1)
		return Py_None;

	int i;
	for(i = 0; i < 3; i++){
		PyObject * item = Py_BuildValue("f", baseSpeed[i]);
		PyList_SET_ITEM(pylist, i, item);
	}

	return pylist;
}

static PyMethodDef i2cMethods[] = {
	{"initI2C", initI2C, METH_VARARGS},
	{"openPort", openPort, METH_VARARGS},
	{"jointReset", jointReset, METH_VARARGS},
	{"jointSetPWM", jointSetPWM, METH_VARARGS},
	{"jointSetSpeed", jointSetSpeed, METH_VARARGS},
	{"jointSetPower", jointSetPower, METH_VARARGS},
	{"baseReset", baseReset, METH_VARARGS},
	{"baseSetSpeed", baseSetSpeed, METH_VARARGS},
	{"getVelPIDConst", getVelPIDConst, METH_VARARGS},
	{"setVelPIDConst", setVelPIDConst, METH_VARARGS},
	{"setPosPIDConst", setPosPIDConst, METH_VARARGS},
	{"setControlMode", setControlMode, METH_VARARGS},
	{"getPosPIDConst", getPosPIDConst, METH_VARARGS},
	{"jointGetSpeed", jointGetSpeed, METH_VARARGS},
	{"jointGetPos", jointGetPos, METH_VARARGS},
	{"jointGetPWM", jointGetPWM, METH_VARARGS},
	{"jointGetPower", jointGetPower, METH_VARARGS},
	{"jointGetRefSpeed", jointGetRefSpeed, METH_VARARGS},
	{"baseGetPos", baseGetPos, METH_VARARGS},
	{"baseGetSpeed", baseGetSpeed, METH_VARARGS},
	{NULL, NULL, 0, NULL}        /* Sentinel */
};


PyMODINIT_FUNC initi2cmodule(void)
{
    (void) Py_InitModule("i2cmodule", i2cMethods);
}



/*****************************************************
	C/C++ Functions
		for Raspberry Pi 2 SMbus-I2C and mecanum base 
******************************************************/

/*********************
	Set() Functions
*********************/
int joint_reset(){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_RESET_JOINT;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	
	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);

	return err;	
}

int joint_setPWM(float fl_pwm, float fr_pwm, float rl_pwm, float rr_pwm){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_JOINT_PWM;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &fl_pwm, sizeof(fl_pwm));
	memcpy(&buff[5], &fr_pwm, sizeof(fr_pwm));
	memcpy(&buff[9], &rl_pwm, sizeof(rl_pwm));
	memcpy(&buff[13], &rr_pwm, sizeof(rr_pwm));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;
}

int joint_setSpeed(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_JOINT_SPEED;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &fl_rpm, sizeof(fl_rpm));
	memcpy(&buff[5], &fr_rpm, sizeof(fr_rpm));
	memcpy(&buff[9], &rl_rpm, sizeof(rl_rpm));
	memcpy(&buff[13], &rr_rpm, sizeof(rr_rpm));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;	
}

int joint_setPower(float fl_pwr, float fr_pwr, float rl_pwr, float rr_pwr){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_JOINT_POWER;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &fl_pwr, sizeof(fl_pwr));
	memcpy(&buff[5], &fr_pwr, sizeof(fr_pwr));
	memcpy(&buff[9], &rl_pwr, sizeof(rl_pwr));
	memcpy(&buff[13], &rr_pwr, sizeof(rr_pwr));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;	
}

int base_reset(void){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_RESET_BASE;
	memcpy(&buff[0], &CMD, sizeof(CMD));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;
}

int base_start(void){
	return 0;
}

int base_stop(void){
	return 0;
}

int base_setSpeed(float longVel, float tranVel, float angVel){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_BASE_SPEED;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &longVel, sizeof(longVel));
	memcpy(&buff[5], &tranVel, sizeof(tranVel));
	memcpy(&buff[9], &angVel, sizeof(angVel));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;	
}

int base_setGoal(void){
	return 0;
}

int set_ControlMode(uint8_t mode){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_CONT_MODE;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &mode, sizeof(mode));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;	
}

int set_VelPIDConst(uint8_t motorNum, float PConst, float IConst, float DConst){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_VEL_PID_CONST;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &motorNum, sizeof(motorNum));
	memcpy(&buff[2], &PConst, sizeof(PConst));
	memcpy(&buff[6], &IConst, sizeof(IConst));
	memcpy(&buff[10], &DConst, sizeof(DConst));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;
}

int set_PosPIDConst(uint8_t mode, float PConst, float IConst, float DConst){
	uint8_t buff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_SET_POS_PID_CONST;
	memcpy(&buff[0], &CMD, sizeof(CMD));
	memcpy(&buff[1], &mode, sizeof(mode));
	memcpy(&buff[2], &PConst, sizeof(PConst));
	memcpy(&buff[6], &IConst, sizeof(IConst));
	memcpy(&buff[10], &DConst, sizeof(DConst));

	int err = masterWriteBlock(fd, TXBUFFERSIZE, buff);
	
	return err;
}

/********************************
	Get() Functions
********************************/

int get_VelPIDConst(uint8_t motorNum, float PIDConsts[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_VEL_PID_CONST;	
	memset(&txBuff[0], CMD, sizeof(CMD));
	memset(&txBuff[1], motorNum, sizeof(motorNum));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&PIDConsts[0], &rxBuffer[0], sizeof(float));	
	memcpy(&PIDConsts[1], &rxBuffer[4], sizeof(float));	
	memcpy(&PIDConsts[2], &rxBuffer[8], sizeof(float));	

	return 0;
}

int get_PosPIDConst(uint8_t mode, float PIDConsts[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_POS_PID_CONST;	
	memset(&txBuff[0], CMD, sizeof(CMD));
	memset(&txBuff[1], mode, sizeof(mode));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&PIDConsts[0], &rxBuffer[0], sizeof(float));	
	memcpy(&PIDConsts[1], &rxBuffer[4], sizeof(float));	
	memcpy(&PIDConsts[2], &rxBuffer[8], sizeof(float));	

	return 0;
}

int joint_getPWM(float jointPWM[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_JOINT_PWM;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&jointPWM[0], &rxBuffer[0], sizeof(float));	
	memcpy(&jointPWM[1], &rxBuffer[4], sizeof(float));	
	memcpy(&jointPWM[2], &rxBuffer[8], sizeof(float));	
	memcpy(&jointPWM[3], &rxBuffer[12], sizeof(float));

	return 0;
}

int joint_getPos(int32_t jointPos[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_JOINT_POS;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&jointPos[0], &rxBuffer[0], sizeof(int32_t));	
	memcpy(&jointPos[1], &rxBuffer[4], sizeof(int32_t));	
	memcpy(&jointPos[2], &rxBuffer[8], sizeof(int32_t));	
	memcpy(&jointPos[3], &rxBuffer[12], sizeof(int32_t));

	return 0;

}

int joint_getSpeed(float jointSpeed[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_JOINT_SPEED;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&jointSpeed[0], &rxBuffer[0], sizeof(float));	
	memcpy(&jointSpeed[1], &rxBuffer[4], sizeof(float));	
	memcpy(&jointSpeed[2], &rxBuffer[8], sizeof(float));	
	memcpy(&jointSpeed[3], &rxBuffer[12], sizeof(float));

	return 0;
}

int joint_getPower(float jointPower[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_JOINT_POWER;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&jointPower[0], &rxBuffer[0], sizeof(float));	
	memcpy(&jointPower[1], &rxBuffer[4], sizeof(float));	
	memcpy(&jointPower[2], &rxBuffer[8], sizeof(float));	
	memcpy(&jointPower[3], &rxBuffer[12], sizeof(float));

	return 0;
}

int joint_getRefSpeed(float jointRefSpeed[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_JOINT_REF_SPEED;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&jointRefSpeed[0], &rxBuffer[0], sizeof(float));	
	memcpy(&jointRefSpeed[1], &rxBuffer[4], sizeof(float));	
	memcpy(&jointRefSpeed[2], &rxBuffer[8], sizeof(float));	
	memcpy(&jointRefSpeed[3], &rxBuffer[12], sizeof(float));

	return 0;
}

int base_getPos(float basePos[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_BASE_POS;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&basePos[0], &rxBuffer[0], sizeof(float));	
	memcpy(&basePos[1], &rxBuffer[4], sizeof(float));	
	memcpy(&basePos[2], &rxBuffer[8], sizeof(float));	

	return 0;
}

int base_getSpeed(float baseSpeed[]){
	uint8_t txBuff[TXBUFFERSIZE] = {0};
	uint8_t CMD = CMD_GET_BASE_SPEED;	
	memset(&txBuff[0], CMD, sizeof(CMD));

	masterWriteBlock(fd, TXBUFFERSIZE, txBuff);	

	uint8_t rxBuffer[RXBUFFERSIZE];	

	masterReadBlock(fd, RXBUFFERSIZE, rxBuffer);

	memcpy(&baseSpeed[0], &rxBuffer[0], sizeof(float));	
	memcpy(&baseSpeed[1], &rxBuffer[4], sizeof(float));	
	memcpy(&baseSpeed[2], &rxBuffer[8], sizeof(float));	

	return 0;
}

int base_getGoal(void){
	return 0;
}

/********************************
	SMbus I2C
********************************/

int init_I2C(void){
	const char *device;

	//RPI-1: "/dev/i2c-0"
	//RPI-2: "/dev/i2c-1"
	int piRev = 2;
	
	if(piRev == 1)
 		device = "/dev/i2c-0";
	else
		device = "/dev/i2c-1";


	if ((fd = open (device, O_RDWR)) < 0){
    		printf( "Unable to open I2C device \n");
		return -1;
	}
	

	return fd;
}

int open_Port(int devId){
	if(ioctl (fd, I2C_SLAVE, devId) < 0){
		printf("Unable to select I2C device \n");
		return -1;
	}
	return 0;
}

inline int masterWriteBlock(int file, uint8_t size, uint8_t buff[]){
	//Register: 0x00 <- Master is writing
	if(i2c_smbus_write_i2c_block_data(file, 0x00, size, buff) < 0){
		printf("i2c master-write failed \n");
		return -1;
	}
	return 0;
}

inline int masterReadBlock(int file, uint8_t size, uint8_t buff[]){
	//Register: 0x01 <- Master is reading
	if(i2c_smbus_read_i2c_block_data(file, 0x01, size, buff) < 0){
		printf("i2c master-read failed \n");
		return -1;
	}
	return 0;
}
