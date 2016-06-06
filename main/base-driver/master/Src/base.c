#include "base.h"

//Resets every wheel with @.reset()
//Sets mode to Velocity Control
//W0.reset() - W1.reset() - W2.reset() - W3.reset()
//@param Base
void base_reset(Base base){
	motor_reset(base.frontLeftWheel);
	motor_reset(base.frontRightWheel);
	motor_reset(base.rearLeftWheel);
	motor_reset(base.rearRightWheel);
}

//Gets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void base_getPosition(Base* base, float* longitudinalPosition, float* transversalPosition, float* orientation){
	int32_t encoderPositions[4] = {0};
	encoderPositions[0] = motor_getPos(base->frontLeftWheel);
  encoderPositions[1] = motor_getPos(base->frontRightWheel);
  encoderPositions[2] = motor_getPos(base->rearLeftWheel);
  encoderPositions[3] = motor_getPos(base->rearRightWheel);
	
  wheelPositionsToCartesianPosition(encoderPositions, base->lastEncoderPositions, longitudinalPosition, transversalPosition, orientation );
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
void base_calcWheelTorques(Base* base, float volatile wheelTorques[]){
	float baseLongitudinalForce = base->controlLong;
	float baseTransversalForce = base->controlTrans;
	float baseOrientationForce = base->controlOrien;
	
	calcJacobianT(&base->controlLong, &base->controlTrans, &base->controlOrien, wheelTorques);
}
