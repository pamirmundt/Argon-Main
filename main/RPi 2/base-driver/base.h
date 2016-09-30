#ifndef BASE_H
#define BASE_H

#include <Python.h>
#include "motor.h"


typedef struct{
	//Base control mode
	//Manuel mode: 		0x01
	//Velocity mode: 	0x02
	//Position mode: 	0x03
	uint8_t controlMode;
	
	//Base motors
	Motor frontLeftWheel;
	Motor frontRightWheel;
	Motor rearLeftWheel;
	Motor rearRightWheel;
	
	volatile float longitudinalPosition;
	volatile float transversalPosition;
	volatile float orientation;

	volatile float longitudinalVelocity;
	volatile float transversalVelocity;
	volatile float angularVelocity;

	volatile float refLongitudinalVelocity;
	volatile float refTransversalVelocity;
	volatile float refAngularVelocity;
	
	//PID
	//Longitudinal PID 
	volatile float KLp;
	volatile float KLi;
	volatile float KLd;
	volatile float refLongitudinalPosition;
	volatile float integralLong;
	volatile float derivativeLong;
	volatile float controlLong;
	volatile float errLong;
	volatile float errPrevLong;
	
	//Transversal PID
	volatile float KTp;
	volatile float KTi;
	volatile float KTd;
	volatile float refTransversalPosition;
	volatile float integralTrans;
	volatile float derivativeTrans;
	volatile float controlTrans;
	volatile float errTrans;
	volatile float errPrevTrans;
	
	//Orientation PID
	volatile float KOp;
	volatile float KOi;
	volatile float KOd;
	volatile float refOrientation;
	volatile float integralOrien;
	volatile float derivativeOrien;
	volatile float controlOrien;
	volatile float errOrien;
	volatile float errPrevOrien;

	int32_t lastEncoderPositions[4];
	volatile float wheelTorques[4];
}Base;

//Initialize mecanum base, needs to called before everything
//	- Attacts wheel to the base
//	- Sets i2c slave-addresses
//	- Sets control gain constants
//	- Sets the default control mode to "velocity mode"
//@param pointer to mecanum base
//@param pointer to front left wheel
//@param pointer to front right wheel
//@param pointer to rear left wheel
//@param pointer to rear right wheel
void base_init(Base* myBase, Motor* FLwheel, Motor* FRwheel, Motor* RLwheel, Motor* RRwheel);

//Resets base variables - Reference velocity, base position, wheel positions, control variables and etc.
//DOES NOT reset control gain constants!
//@param pointer to mecanum base
void base_reset(Base* myBase);

void base_setGoal(void); //Not yet implemented

//Get base position on x,y and theta in meters and radians
//@pointer to mecanum base
//@float array (size:3) which will be filled with base positions
void base_getPosition(Base* myBase, float positions[]);

void base_getGoal(void);	//Not yet implemented

//Set Control mode
//	0: Manuel mode - allows to set wheelPWM, wheelSpeed, wheelPower and etc. manually
//	1: Velocity mode - only velocity control on slaves are working
//	2: Position mode - velocity control on slaves and position control on master are working
//@param pointer to mecanum base
//@param control mode
void base_setControlMode(Base* myBase, uint8_t mode);

//Set Position PID Control Constants
//Position control is taken careof by the master
//@param pointer to mecanum base
//@param 	mode:0 - Longitudal PID constant
//				mode:1 - Tranversal PID constant
//				mode:2 - Angular PID constant
//@param Proportional constant
//@param Integral constant
//@param Derivative constant
void setPositionPID(Base* myBase, uint8_t mode, float Kp, float Ki, float Kd);

//Set Ve1ocity PID Control Constants
//Velocity control is take careof by each slave
//@param pointer to mecanum base
//@param 	motor:0 - front left motor
//				motor:1 - front right motor
//				motor:2 - rear left motor
//				motor:3 - rear right motor
void setVelocityPID(Base* myBase, uint8_t motor, float Kp, float Ki, float Kd);

//Gets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void base_getUpdatePosition(PyObject *pArgs_wheelPositionsToCartesianPosition, PyObject *pFunc_wheelPositionsToCartesianPosition, Base* base);

//Gets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_getVelocity(PyObject *pArgs_wheelVelocitiesToCartesianVelocity, PyObject *pFunc_wheelVelocitiesToCartesianVelocity, Base* base);

//Sets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_setVelocity(PyObject *pArgs_cartesianVelocityToWheelVelocities, PyObject *pFunc_cartesianVelocityToWheelVelocities, Base base, float longitudinalVelocity, float transversalVelocity, float angularVelocity);

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base
//@param returns wheel torques
void base_calcWheelTorques(PyObject *pArgs_calcJacobianT, PyObject *pFunc_calcJacobianT, Base* base);

#endif
