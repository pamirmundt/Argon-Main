#include "Base.h"

//Base constructor
Base::Base(Wheel _frontLeftWheel, Wheel _frontRightWheel, Wheel _rearLeftWheel, Wheel _rearRightWheel){
  frontLeftWheel = _frontLeftWheel;
  frontRightWheel = _frontRightWheel;
  rearLeftWheel = _rearLeftWheel;
  rearRightWheel = _rearRightWheel;
}

//Resets every wheel with @.reset()
//W0.reset() - W1.reset() - W2.reset() - W3.reset()
void Base::reset(){
  frontLeftWheel.reset();
  frontRightWheel.reset();
  rearLeftWheel.reset();
  rearRightWheel.reset();
}

//Sets the control mode of the base
//0 - Manuel Mode (setPower, setPWM, etc.)
//1 - Position Control Mode with PID
//2 - Velocity Control Mode with PID
void Base::setMode(uint8_t mode){
  frontLeftWheel.setMode(mode);
  frontRightWheel.setMode(mode);
  rearLeftWheel.setMode(mode);
  rearRightWheel.setMode(mode);
}

//Gets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void Base::getBasePosition(float& longitudinalPosition, float& transversalPosition, float& orientation){
  int32_t encoderPositions[4] = {0};
  encoderPositions[0] = frontLeftWheel.getPos();
  encoderPositions[1] = frontRightWheel.getPos();
  encoderPositions[2] = rearLeftWheel.getPos();
  encoderPositions[3] = rearRightWheel.getPos();

  wheelPositionsToCartesianPosition( encoderPositions, lastEncoderPositions, longitudinalPosition, transversalPosition, orientation );
}

//Sets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void Base::setBasePosition(float longitudinalPosition, float transversalPosition, float angularPosition, float intervalTime){
  //Derivative of trajectory
  float longitudinalVelocity = (longitudinalPosition - prevLongitudinalPosition) / intervalTime;
  float transversalVelocity = (transversalPosition - prevTransversalPosition) / intervalTime;
  float angularVelocity = (angularPosition - prevAngularPosition) / intervalTime;

  prevLongitudinalPosition = longitudinalPosition;
  prevTransversalPosition = transversalPosition;
  prevAngularPosition = angularPosition;
  
  float W0_RPM, W1_RPM, W2_RPM, W3_RPM;
  cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, W0_RPM, W1_RPM, W2_RPM, W3_RPM);

  frontLeftWheel.moveSpeed(W0_RPM);
  frontRightWheel.moveSpeed(W1_RPM);
  rearLeftWheel.moveSpeed(W2_RPM);
  rearRightWheel.moveSpeed(W3_RPM);
}

//Gets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void Base::getBaseVelocity(float& longitudinalVelocity, float& transversalVelocity, float& angularVelocity){
  float RPM0 = frontLeftWheel.getSpeed();
  float RPM1 = frontRightWheel.getSpeed();
  float RPM2 = rearLeftWheel.getSpeed();
  float RPM3 = rearRightWheel.getSpeed();
  wheelVelocitiesToCartesianVelocity(RPM0, RPM1, RPM2, RPM3, longitudinalVelocity, transversalVelocity, angularVelocity);
}

//Sets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void Base::setBaseVelocity(float longitudinalVelocity, float transversalVelocity, float angularVelocity){
  float W0_RPM, W1_RPM, W2_RPM, W3_RPM;
  cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, W0_RPM, W1_RPM, W2_RPM, W3_RPM);
  
  frontLeftWheel.moveSpeed(W0_RPM);
  frontRightWheel.moveSpeed(W1_RPM);
  rearLeftWheel.moveSpeed(W2_RPM);
  rearRightWheel.moveSpeed(W3_RPM);
}

