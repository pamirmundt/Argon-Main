#ifndef BASE_H
#define BASE_H

#include "BaseKinematics.h"
#include "motor.h"

extern const float gearRatio;

typedef struct{
	//Base motors
	Motor frontLeftWheel;
	Motor frontRightWheel;
	Motor rearLeftWheel;
	Motor rearRightWheel;
	
	float longitudinalPosition;
	float transversalPosition;
	float orientation;
	
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

	volatile int32_t lastEncoderPositions[4];
	volatile float wheelTorques[4];
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
void base_getPosition(Base* base, float* longitudinalPosition, float* transversalPosition, float* orientation);

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

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base
//@param returns wheel torques
void base_calcWheelTorques(Base* base, volatile float wheelTorques[]);

#endif
