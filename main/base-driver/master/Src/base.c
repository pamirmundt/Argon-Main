#include "base.h"

void base_reset(Base base){
	motor_reset(base.frontLeftWheel);
	motor_reset(base.frontRightWheel);
	motor_reset(base.rearLeftWheel);
	motor_reset(base.rearRightWheel);
}

void base_getPosition(Base base, float* longitudinalPosition, float* transversalPosition, float* orientation){
  uint32_t encoderPositions[4] = {0};
	
	encoderPositions[0] = motor_getPos(base.frontLeftWheel);
  encoderPositions[1] = motor_getPos(base.frontRightWheel);
  encoderPositions[2] = motor_getPos(base.rearLeftWheel);
  encoderPositions[3] = motor_getPos(base.rearRightWheel);

  wheelPositionsToCartesianPosition(encoderPositions, base.lastEncoderPositions, longitudinalPosition, transversalPosition, orientation );
}

void base_setPosition(Base base, float longitudinalPosition, float transversalPosition, float angularPosition, float intervalTime){
  //Derivative of trajectory
  float longitudinalVelocity = (longitudinalPosition - base.prevLongitudinalPosition) / intervalTime;
  float transversalVelocity = (transversalPosition - base.prevTransversalPosition) / intervalTime;
  float angularVelocity = (angularPosition - base.prevAngularPosition) / intervalTime;

  base.prevLongitudinalPosition = longitudinalPosition;
  base.prevTransversalPosition = transversalPosition;
  base.prevAngularPosition = angularPosition;
  
  float W0_RPM, W1_RPM, W2_RPM, W3_RPM;
  cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, &W0_RPM, &W1_RPM, &W2_RPM, &W3_RPM);

	motor_setRPM(base.frontLeftWheel, W0_RPM);
	motor_setRPM(base.frontRightWheel, W1_RPM);
	motor_setRPM(base.rearLeftWheel, W2_RPM);
	motor_setRPM(base.rearRightWheel, W3_RPM);
}

void base_getVelocity(Base base, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity){
	float RPM0 = motor_getSpeed(base.frontLeftWheel);
  float RPM1 = motor_getSpeed(base.frontRightWheel);
  float RPM2 = motor_getSpeed(base.rearLeftWheel);
  float RPM3 = motor_getSpeed(base.rearRightWheel);
	
  wheelVelocitiesToCartesianVelocity(RPM0, RPM1, RPM2, RPM3, longitudinalVelocity, transversalVelocity, angularVelocity);
}

void base_setVelocity(Base base, float longitudinalVelocity, float transversalVelocity, float angularVelocity){
	float W0_RPM, W1_RPM, W2_RPM, W3_RPM;
  cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, &W0_RPM, &W1_RPM, &W2_RPM, &W3_RPM);
  
	motor_setRPM(base.frontLeftWheel, W0_RPM);
	motor_setRPM(base.frontRightWheel, W1_RPM);
	motor_setRPM(base.rearLeftWheel, W2_RPM);
	motor_setRPM(base.rearRightWheel, W3_RPM);
}
