//NOT USED ANYMORE
//C FUNCTIONS ARE NOW PYHTON FUNCTIONS

#ifndef BASEKINEMATICS_H
#define BASEKINEMATICS_H

#include "stdint.h"

#define gearRatio                         13.552f //1:13.552 Ratio

//set base velocity
//Calculates from the cartesian velocity the individual wheel velocities 
//@param longitudinalVelocity is the forward or backward velocity
//@param transversalVelocity is the sideway velocity
//@param angularVelocity is the rotational velocity around the center
//@param wheelVelocities are the individual wheel velocities
void cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity, float* W0_RPM, float* W1_RPM, float* W2_RPM, float* W3_RPM);

//get base velocity
//Calculates from the wheel velocities the cartesian velocity
//@param wheelVelocities are the velocities of the individual wheels
//@param longitudinalVelocity is the forward or backward velocity
//@param transversalVelocity is the sideway velocity
//@param angularVelocity is the rotational velocity around the center
void wheelVelocitiesToCartesianVelocity(float W0_RPM, float W1_RPM, float W2_RPM, float W3_RPM, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity);

//get base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
void wheelPositionsToCartesianPosition(int32_t encoderPositions[], int32_t lastEncoderPositions[], float* longitudinalPosition , float* transversalPosition, float* orientation );

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base longitudinal force
//@param base transversal force
//@param base orientation force
//@param returns wheel torques
void calcJacobianT(volatile float* baseLongitudinalForce, volatile float* baseTransversalForce, volatile float* baseOrientationForce, volatile float wheelTorques[]);

#endif
