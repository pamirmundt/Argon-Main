#include "base.h"

/**
  * @brief  Initialize Base
							- Reset wheels
							-	Reset base parameters
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
	HAL_TIM_Base_Stop_IT(&htim9);
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
			//Disable Position Control
			HAL_TIM_Base_Stop_IT(&htim9);
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
			//Disable Position Mode
			HAL_TIM_Base_Stop_IT(&htim9);
			break;
		
		case 0x02:
			//Enable Velocity Mode
			motor_setMode(myBase->frontLeftWheel, 1);
			motor_setMode(myBase->frontRightWheel, 1);
			motor_setMode(myBase->rearLeftWheel, 1);
			motor_setMode(myBase->rearRightWheel, 1);
			//Enable Position Mode
			HAL_TIM_Base_Start_IT(&htim9);
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
void base_getUpdatePosition(Base* base, float* longitudinalPosition, float* transversalPosition, float* orientation){
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
