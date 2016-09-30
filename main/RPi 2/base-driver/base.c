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
void base_getUpdatePosition(PyObject *pArgs_wheelPositionsToCartesianPosition, PyObject *pFunc_wheelPositionsToCartesianPosition, Base* base){
	//Fill Python Tuple
	//encPos0, encPos1, encPos2, encPos3, lastencPos0, lastencPos1, lastencPos2, lastencPos3, longitudinalPosition, transversalPosition, orientation
	//Encoder Positions
	PyObject *pValue;

	pValue = PyLong_FromLong(base->frontLeftWheel.encoderPosition);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 0, pValue);
	pValue = PyLong_FromLong(base->frontRightWheel.encoderPosition);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 1, pValue);
	pValue = PyLong_FromLong(base->rearLeftWheel.encoderPosition);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 2, pValue);
	pValue = PyLong_FromLong(base->rearRightWheel.encoderPosition);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 3, pValue);
	//Last Encoder Positions
	pValue = PyLong_FromLong(base->lastEncoderPositions[0]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 4, pValue);
	pValue = PyLong_FromLong(base->lastEncoderPositions[1]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 5, pValue);
	pValue = PyLong_FromLong(base->lastEncoderPositions[2]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 6, pValue);
	pValue = PyLong_FromLong(base->lastEncoderPositions[3]);
	PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 7, pValue);
	//Mecanum Base Positions
	pValue = PyFloat_FromDouble(base->longitudinalPosition);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 8, pValue);
    pValue = PyFloat_FromDouble(base->transversalPosition);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 9, pValue);
    pValue = PyFloat_FromDouble(base->orientation);
    PyTuple_SetItem(pArgs_wheelPositionsToCartesianPosition, 10, pValue);

    //Execute pyhton function "wheelPositionsToCartesianPosition"
    //Arguments: lastEncPos0, lastEncPos1, lastEncPos2, lastEncPos3, longitudinalPosition, transversalPosition, orientation)
    PyObject *pRetValue = PyObject_CallObject(pFunc_wheelPositionsToCartesianPosition, pArgs_wheelPositionsToCartesianPosition);

    //Return Arguments
    //Last Encoder Positions
	base->lastEncoderPositions[0] = PyLong_AsLong(PyTuple_GetItem(pRetValue, 0));
	base->lastEncoderPositions[1] = PyLong_AsLong(PyTuple_GetItem(pRetValue, 1));
	base->lastEncoderPositions[2] = PyLong_AsLong(PyTuple_GetItem(pRetValue, 2));
	base->lastEncoderPositions[3] = PyLong_AsLong(PyTuple_GetItem(pRetValue, 3));
	//Base Positions
	base->longitudinalPosition = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 4));
	base->transversalPosition = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 5));
	base->orientation = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 6));

	//Py_CLEAR(pValue);
	Py_CLEAR(pRetValue);
}

//Gets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_getVelocity(PyObject *pArgs_wheelVelocitiesToCartesianVelocity, PyObject *pFunc_wheelVelocitiesToCartesianVelocity, Base* base){
	PyObject *pValue;

  	pValue = PyFloat_FromDouble(base->frontLeftWheel.RPM);
	PyTuple_SetItem(pArgs_wheelVelocitiesToCartesianVelocity, 0, pValue);
	pValue = PyFloat_FromDouble(base->frontRightWheel.RPM);
	PyTuple_SetItem(pArgs_wheelVelocitiesToCartesianVelocity, 1, pValue);
	pValue = PyFloat_FromDouble(base->rearLeftWheel.RPM);
	PyTuple_SetItem(pArgs_wheelVelocitiesToCartesianVelocity, 2, pValue);
	pValue = PyFloat_FromDouble(base->rearRightWheel.RPM);
	PyTuple_SetItem(pArgs_wheelVelocitiesToCartesianVelocity, 3, pValue);
	
	//Execute pyhton function "wheelVelocitiesToCartesianVelocity"
	PyObject *pRetValue = PyObject_CallObject(pFunc_wheelVelocitiesToCartesianVelocity, pArgs_wheelVelocitiesToCartesianVelocity);

	base->longitudinalVelocity = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 0));
	base->transversalVelocity = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 1));
	base->angularVelocity = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 2));

	//Py_CLEAR(pValue);
	Py_CLEAR(pRetValue);
}

//Sets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_setVelocity(PyObject *pArgs_cartesianVelocityToWheelVelocities, PyObject *pFunc_cartesianVelocityToWheelVelocities, Base base, float longitudinalVelocity, float transversalVelocity, float angularVelocity){
	PyObject *pValue;

	pValue = PyFloat_FromDouble(longitudinalVelocity);
	PyTuple_SetItem(pArgs_cartesianVelocityToWheelVelocities, 0, pValue);
	pValue = PyFloat_FromDouble(transversalVelocity);
	PyTuple_SetItem(pArgs_cartesianVelocityToWheelVelocities, 1, pValue);
	pValue = PyFloat_FromDouble(angularVelocity);
	PyTuple_SetItem(pArgs_cartesianVelocityToWheelVelocities, 2, pValue);

	//Execute pyhton function "cartesianVelocityToWheelVelocities"
	PyObject *pRetValue = PyObject_CallObject(pFunc_cartesianVelocityToWheelVelocities, pArgs_cartesianVelocityToWheelVelocities);

	motor_setRPM(base.frontLeftWheel, PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 0)));
	motor_setRPM(base.frontRightWheel, PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 1)));
	motor_setRPM(base.rearLeftWheel, PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 2)));
	motor_setRPM(base.rearRightWheel, PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 3)));

	//Py_CLEAR(pValue);
	Py_CLEAR(pRetValue);
}

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base
//@param returns wheel torques
void base_calcWheelTorques(PyObject *pArgs_calcJacobianT, PyObject *pFunc_calcJacobianT, Base* base){
	PyObject *pValue;

	pValue = PyFloat_FromDouble(base->controlLong);
	PyTuple_SetItem(pArgs_calcJacobianT, 0, pValue);
	pValue = PyFloat_FromDouble(base->controlTrans);
	PyTuple_SetItem(pArgs_calcJacobianT, 1, pValue);
	pValue = PyFloat_FromDouble(base->controlOrien);
	PyTuple_SetItem(pArgs_calcJacobianT, 2, pValue);

	//Execute pyhton function "calcJacobianT"
    //Arguments: Longitudinal Force, Transversal Force, Orientation Force
    PyObject *pRetValue = PyObject_CallObject(pFunc_calcJacobianT, pArgs_calcJacobianT);
	
	//Return Arguments
    //Left Front Wheel Torque (W0), Left Right Wheel Torque (W1), Rear Left Wheel Torque (W2), Rear Right Wheel Torque (W3)
	base->wheelTorques[0] = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 0));
	base->wheelTorques[1] = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 1));
	base->wheelTorques[2] = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 2));
	base->wheelTorques[3] = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 3));

	//Py_CLEAR(pValue);
	Py_CLEAR(pRetValue);
}
