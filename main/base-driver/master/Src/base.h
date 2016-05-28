#ifndef BASE_H
#define BASE_H

#include "BaseKinematics.h"
#include "motor.h"

typedef struct{
	Motor frontLeftWheel;
	Motor frontRightWheel;
	Motor rearLeftWheel;
	Motor rearRightWheel;

	int32_t lastEncoderPositions[4];
	//float longitudinalPosition;
	//float transversalPosition;
	//float orientation;
	float prevLongitudinalPosition;
	float prevTransversalPosition;
	float prevAngularPosition;
}Base;

//Resets every wheel with @.reset()
//Sets mode to Velocity Control
//W0.reset() - W1.reset() - W2.reset() - W3.reset()
//@param Base
void base_reset(Base base);

//Gets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
void base_getPosition(Base base, float* longitudinalPosition, float* transversalPosition, float* orientation);

//Sets the cartesian base position
//@param longitudinalPosition - forward/backward position
//@param transversalPosition - sideway position
//@param orientation - orientation
//void base_setPosition(float longitudinalPosition, float transversalPosition, float angularPosition, float intervalTime);

//Gets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_getVelocity(Base base, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity);

//Sets the cartesian base velocity
//@param longitudinalVelocity - forward/backward velocity
//@param transversalVelocity - sideway velocity
//@param angularVelocity - rotational velocity
void base_setVelocity(Base base, float longitudinalVelocity, float transversalVelocity, float angularVelocity);

#endif
