#include <string.h>
#include "base.h"

/**
  * @brief  Initialize Base
			- Reset wheels
			- Reset base parameters
  * @param  None
  * @retval None
  */
void base_init(Base* myBase, Motor* FLwheel, Motor* FRwheel, Motor* RLwheel, Motor* RRwheel){
	//Reset base instance
	memset(myBase, 0, sizeof(Base));
	
	//Set motors
	myBase->frontLeftWheel = *FLwheel;
	myBase->frontRightWheel = *FRwheel;
	myBase->rearLeftWheel = *RLwheel;
	myBase->rearRightWheel = *RRwheel;
	
	//Reset Wheels
	motor_reset(myBase->frontLeftWheel);
	motor_reset(myBase->frontRightWheel);
	motor_reset(myBase->rearLeftWheel);
	motor_reset(myBase->rearRightWheel);
	
	//Set PID constants
	//Longitudinal PID Constants
	myBase->KLp = KLp;
	myBase->KLi = KLi;
	myBase->KLd = KLd;
	
	//Transversal PID Constants
	myBase->KTp = KTp;
	myBase->KTi = KTi;
	myBase->KTd = KTd;
	
	//Orientation PID Constants
	myBase->KOp = KOp;
	myBase->KOi = KOi;
	myBase->KOd = KOd;

	//Default Mode: Velocity Control
	myBase->controlMode = 0x01;
}

void base_reset(Base* myBase){	
	myBase->longitudinalPosition = 0.0f;
	myBase->transversalPosition = 0.0f;
	myBase->orientation = 0.0f;

	myBase->refLongitudinalPosition = 0.0f;
	myBase->integralLong = 0.0f;
	myBase->derivativeLong = 0.0f;
	myBase->controlLong = 0.0f;
	myBase->errLong = 0.0f;
	myBase->errPrevLong = 0.0f;
	
	myBase->refTransversalPosition = 0.0f;
	myBase->integralTrans = 0.0f;
	myBase->derivativeTrans = 0.0f;
	myBase->controlTrans = 0.0f;
	myBase->errTrans = 0.0f;
	myBase->errPrevTrans = 0.0f;
	
	myBase->refOrientation = 0.0f;
	myBase->integralOrien = 0.0f;
	myBase->derivativeOrien = 0.0f;
	myBase->controlOrien = 0.0f;
	myBase->errOrien = 0.0f;
	myBase->errPrevOrien = 0.0f;

	myBase->lastEncoderPositions[0] = 0;
	myBase->lastEncoderPositions[1] = 0;
	myBase->lastEncoderPositions[2] = 0;
	myBase->lastEncoderPositions[3] = 0;
	
	myBase->wheelTorques[0] = 0;
	myBase->wheelTorques[1] = 0;
	myBase->wheelTorques[2] = 0;
	myBase->wheelTorques[3] = 0;
}

void base_getPosition(Base* myBase, float positions[]){
	positions[0] = myBase->longitudinalPosition;
	positions[1] = myBase->transversalPosition;
	positions[2] = myBase->orientation;
}

void base_setControlMode(Base* myBase, uint8_t mode){
	//Just reset variables (not wheels)
	//Store wheels
	Motor wheelFL = myBase->frontLeftWheel;  
	Motor wheelFR = myBase->frontRightWheel;
	Motor wheelRL = myBase->rearLeftWheel;
	Motor wheelRR = myBase->rearRightWheel;
	base_init(myBase, &wheelFL, &wheelFR, &wheelRL, &wheelRR);
	
	switch(mode){
		//Manuel mode
		case 0x00:
			//Disable Velocity Control
			motor_setMode(myBase->frontLeftWheel, 0);
			motor_setMode(myBase->frontRightWheel, 0);
			motor_setMode(myBase->rearLeftWheel, 0);
			motor_setMode(myBase->rearRightWheel, 0);
			break;
		
		//Velocity Mode
		case 0x01:
			motor_reset(myBase->frontLeftWheel);
			motor_reset(myBase->frontRightWheel);
			motor_reset(myBase->rearLeftWheel);
			motor_reset(myBase->rearRightWheel);
			//Enable Velocity Mode
			motor_setMode(myBase->frontLeftWheel, 1);
			motor_setMode(myBase->frontRightWheel, 1);
			motor_setMode(myBase->rearLeftWheel, 1);
			motor_setMode(myBase->rearRightWheel, 1);
			break;
		
		case 0x02:
			//Enable Velocity Mode
			motor_setMode(myBase->frontLeftWheel, 1);
			motor_setMode(myBase->frontRightWheel, 1);
			motor_setMode(myBase->rearLeftWheel, 1);
			motor_setMode(myBase->rearRightWheel, 1);
			break;
	}
	myBase->controlMode = mode;
}

void setPositionPID(Base* myBase, uint8_t mode, float Kp, float Ki, float Kd){
	switch (mode){
		case 0x00: {
			myBase->KLp = Kp;
			myBase->KLi = Ki;
			myBase->KLd = Kd;
		} break;
		
		case 0x01: {
			myBase->KTp = Kp;
			myBase->KTi = Ki;
			myBase->KTd = Kd;
		} break;
		
		case 0x02: {
			myBase->KOp = Kp;
			myBase->KOi = Ki;
			myBase->KOd = Kd;
		} break;
	}
}

void setVelocityPID(Base* myBase, uint8_t motor, float Kp, float Ki, float Kd){
	switch (motor){
		case 0x00:{
			motor_setPID(myBase->frontLeftWheel, Kp, Ki, Kd);
		} break;
				
		case 0x01:{
			motor_setPID(myBase->frontRightWheel, Kp, Ki, Kd);
		} break;
		
		case 0x02:{
			motor_setPID(myBase->rearLeftWheel, Kp, Ki, Kd);
		} break;
		
		case 0x03:{
			motor_setPID(myBase->rearRightWheel, Kp, Ki, Kd);
		} break;
	}
}

//Gets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void base_getUpdatePosition(PyObject *pArgs_wheelPositionsToCartesianPosition, PyObject *pKinematicsValue, PyObject *pFunc_wheelPositionsToCartesianPosition, Base* base, float* longitudinalPosition, float* transversalPosition, float* orientation){
	int32_t encoderPositions[4] = {0};
	encoderPositions[0] = motor_getPos(base->frontLeftWheel);
	encoderPositions[1] = motor_getPos(base->frontRightWheel);
	encoderPositions[2] = motor_getPos(base->rearLeftWheel);
	encoderPositions[3] = motor_getPos(base->rearRightWheel);
	
	//Fill Python Tuple
	//encPos0, encPos1, encPos2, encPos3, lastencPos0, lastencPos1, lastencPos2, lastencPos3, longitudinalPosition, transversalPosition, orientation
	//Encoder Positions
	pKinematicsValue = PyLong_FromLong(encoderPositions[0]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 0, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(encoderPositions[1]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 1, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(encoderPositions[2]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 2, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(encoderPositions[3]);
	//Last Encoder Positions
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 3, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(base->lastEncoderPositions[0]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 4, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(base->lastEncoderPositions[1]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 5, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(base->lastEncoderPositions[2]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 6, pKinematicsValue);
	pKinematicsValue = PyLong_FromLong(base->lastEncoderPositions[3]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 7, pKinematicsValue);
	//Mecanum Base Positions
	pKinematicsValue = PyFloat_FromDouble(*longitudinalPosition);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 8, pKinematicsValue);
    pKinematicsValue = PyFloat_FromDouble(*transversalPosition);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 9, pKinematicsValue);
    pKinematicsValue = PyFloat_FromDouble(*orientation);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 10, pKinematicsValue);

    //Execute pyhton function "wheelPositionsToCartesianPosition"
    //Arguments: lastEncPos0, lastEncPos1, lastEncPos2, lastEncPos3, longitudinalPosition, transversalPosition, orientation)
    pKinematicsValue = PyObject_CallObject(pFunc_wheelPositionsToCartesianPosition, pArgs_wheelPositionsToCartesianPosition);

    //Return Arguments
    //Last Encoder Positions
	base->lastEncoderPositions[0] = PyLong_AsLong(PyTuple_GetItem(pKinematicsValue, 0));
	base->lastEncoderPositions[1] = PyLong_AsLong(PyTuple_GetItem(pKinematicsValue, 1));
	base->lastEncoderPositions[2] = PyLong_AsLong(PyTuple_GetItem(pKinematicsValue, 2));
	base->lastEncoderPositions[3] = PyLong_AsLong(PyTuple_GetItem(pKinematicsValue, 3));
	//Base Positions
	base->longitudinalPosition = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 4));
	base->transversalPosition = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 5));
	base->orientation = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 6));

}

//Gets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_getVelocity(Base base, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity){
	float RPM0 = motor_getSpeed(base.frontLeftWheel) / gearRatio;
	float RPM1 = motor_getSpeed(base.frontRightWheel) / gearRatio;
	float RPM2 = motor_getSpeed(base.rearLeftWheel) / gearRatio;
  	float RPM3 = motor_getSpeed(base.rearRightWheel) /gearRatio;
	
  	wheelVelocitiesToCartesianVelocity(RPM0, RPM1, RPM2, RPM3, longitudinalVelocity, transversalVelocity, angularVelocity);
}

//Sets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_setVelocity(Base base, float longitudinalVelocity, float transversalVelocity, float angularVelocity){
	float W0_RPM, W1_RPM, W2_RPM, W3_RPM;
  	cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, &W0_RPM, &W1_RPM, &W2_RPM, &W3_RPM);
  
	motor_setRPM(base.frontLeftWheel, W0_RPM);
	motor_setRPM(base.frontRightWheel, W1_RPM);
	motor_setRPM(base.rearLeftWheel, W2_RPM);
	motor_setRPM(base.rearRightWheel, W3_RPM);
}

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base
//@param returns wheel torques
void base_calcWheelTorques(PyObject *pArgs_calcJacobianT, PyObject *pKinematicsValue, PyObject *pFunc_calcJacobianT, Base* base){
	pKinematicsValue = PyFloat_FromDouble(base->controlLong);
	PyTuple_SetItem(pArgs_calcJacobianT, 0, pKinematicsValue);
	pKinematicsValue = PyFloat_FromDouble(base->controlTrans);
	PyTuple_SetItem(pArgs_calcJacobianT, 1, pKinematicsValue);
	pKinematicsValue = PyFloat_FromDouble(base->controlOrien);
	PyTuple_SetItem(pArgs_calcJacobianT, 2, pKinematicsValue);

	//Execute pyhton function "calcJacobianT"
    //Arguments: Longitudinal Force, Transversal Force, Orientation Force
    pKinematicsValue = PyObject_CallObject(pFunc_calcJacobianT, pArgs_calcJacobianT);
	
	//Return Arguments
    //Left Front Wheel Torque (W0), Left Right Wheel Torque (W1), Rear Left Wheel Torque (W2), Rear Right Wheel Torque (W3)
	base->wheelTorques[0] = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 0));
	base->wheelTorques[1] = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 1));
	base->wheelTorques[2] = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 2));
	base->wheelTorques[3] = PyFloat_AsDouble(PyTuple_GetItem(pKinematicsValue, 3));

	//calcJacobianT(&base->controlLong, &base->controlTrans, &base->controlOrien, wheelTorques);
}
