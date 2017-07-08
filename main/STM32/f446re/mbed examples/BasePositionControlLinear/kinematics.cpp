/* Includes ------------------------------------------------------------------*/
#include "mbed.h"

#include "base.h"
#include "params.h"
#include "kinematics.h"

/* Defines -------------------------------------------------------------------*/
#define M_PI    3.14159265358979323846f


/* Local Variables ------------------------------------------------------------*/
volatile int16_t lastEncPos1 = 0, lastEncPos2 = 0, lastEncPos3 = 0, lastEncPos4 = 0;


/* Functions -----------------------------------------------------------------*/

//Calculates from the cartesian velocity the individual wheel velocities 
//@param - longitudinalVelocity is the forward or backward velocity (m/s)
//@param - transversalVelocity is the sideway velocity (m/s)
//@param - angularVelocity is the rotational velocity around the center (rad/s)
//@param - wheelVelocities are the individual wheel velocities (RPM)
void cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity){
  float W1_angVel = (longitudinalVelocity + transversalVelocity + geomFactor*angularVelocity)/wheelRadius;
  float W2_angVel = (longitudinalVelocity - transversalVelocity - geomFactor*angularVelocity)/wheelRadius;
  float W3_angVel = (longitudinalVelocity - transversalVelocity + geomFactor*angularVelocity)/wheelRadius;
  float W4_angVel = (longitudinalVelocity + transversalVelocity - geomFactor*angularVelocity)/wheelRadius;

  //Angular velocity to RPM
  wheel1_refRPM = W1_angVel * 60.0f / (2.0f * M_PI);
  wheel2_refRPM = W2_angVel * 60.0f / (2.0f * M_PI);
  wheel3_refRPM = W3_angVel * 60.0f / (2.0f * M_PI);
  wheel4_refRPM = W4_angVel * 60.0f / (2.0f * M_PI);
}

//Calculates from the wheel velocities the cartesian velocity
//@param wheelVelocities are the velocities of the individual wheels (RPM)
//@param longitudinalVelocity is the forward or backward velocity (m/s)
//@param transversalVelocity is the sideway velocity (m/s)
//@param angularVelocity is the rotational velocity around the center (rad/s)
void wheelVelocitiesToCartesianVelocity(float W1_RPM, float W2_RPM, float W3_RPM, float W4_RPM, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity){
  //RPM to rad/s
  float W1_angVel = W1_RPM * 2.0f * M_PI / 60.0f;
  float W2_angVel = W2_RPM * 2.0f * M_PI / 60.0f;
  float W3_angVel = W3_RPM * 2.0f * M_PI / 60.0f;
  float W4_angVel = W4_RPM * 2.0f * M_PI / 60.0f;

  *longitudinalVelocity = (W1_angVel + W2_angVel + W3_angVel + W4_angVel) * wheelRadius / 4.0f;
  *transversalVelocity = (W1_angVel - W2_angVel - W3_angVel + W4_angVel) * wheelRadius /4.0f;
  *angularVelocity = (W1_angVel - W2_angVel + W3_angVel - W4_angVel) * wheelRadius /(4.0f * geomFactor);
}

//Update/get base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
void wheelPositionsToCartesianPosition(){
    //Calculate Delta Encoder Position
    int16_t encPos1 = getEncoderCount(1);
    int16_t encPos2 = getEncoderCount(2);
    int16_t encPos3 = getEncoderCount(3);
    int16_t encPos4 = getEncoderCount(4);
    
    int16_t dPosW1 = encPos1 - lastEncPos1;
    int16_t dPosW2 = encPos2 - lastEncPos2;
    int16_t dPosW3 = encPos3 - lastEncPos3;
    int16_t dPosW4 = encPos4 - lastEncPos4;

    //Store Encoder Count for next iteration
    lastEncPos1 = encPos1;
    lastEncPos2 = encPos2;
    lastEncPos3 = encPos3;
    lastEncPos4 = encPos4;

    //Calculate Delta Longitudial Position in meters
    float deltaLongitudinalPos = (float(dPosW1 + dPosW2 + dPosW3 + dPosW4) * 2.0f * M_PI * wheelRadius) / 4.0f / encoder_resolution / gearRatio / encoder_mode;
    //Calculate Delta Transversal Position in meters
    float deltaTransversalPos = (float(dPosW1 - dPosW2 - dPosW3 + dPosW4) * 2.0f * M_PI * wheelRadius) / 4.0f / encoder_resolution / gearRatio / encoder_mode;

    //Calculate Base Position (x, y, theta)
    //Radians
    orientation += (float(dPosW1 - dPosW2 + dPosW3 - dPosW4) * 2.0f * M_PI * wheelRadius) / 4.0f / geomFactor / encoder_resolution / gearRatio / encoder_mode;

    //Meters
    longitudinalPosition += deltaLongitudinalPos * cos(orientation) - deltaTransversalPos * sin(orientation);

    //Meters
    transversalPosition += deltaLongitudinalPos * sin(orientation) + deltaTransversalPos * cos(orientation);
}

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base longitudinal force
//@param base transversal force
//@param base orientation force
//@param returns wheel torques
void calcJacobianT(float baseLongitudinalForce, float baseTransversalForce, float baseOrientationForce){    
    wheel1Torque = baseLongitudinalForce*(cos(orientation)-sin(orientation))/sqrtf(2.0f) + baseTransversalForce*(sin(orientation)+cos(orientation))/sqrtf(2.0f) + baseOrientationForce*geomFactor*sqrtf(2.0f);
    wheel2Torque = baseLongitudinalForce*(cos(orientation)+sin(orientation))/sqrtf(2.0f) + baseTransversalForce*(sin(orientation)-cos(orientation))/sqrtf(2.0f) - baseOrientationForce*geomFactor*sqrtf(2.0f);
    wheel3Torque = baseLongitudinalForce*(cos(orientation)+sin(orientation))/sqrtf(2.0f) + baseTransversalForce*(sin(orientation)-cos(orientation))/sqrtf(2.0f) + baseOrientationForce*geomFactor*sqrtf(2.0f);
    wheel4Torque = baseLongitudinalForce*(cos(orientation)-sin(orientation))/sqrtf(2.0f) + baseTransversalForce*(sin(orientation)+cos(orientation))/sqrtf(2.0f) - baseOrientationForce*geomFactor*sqrtf(2.0f);

    wheel1_refRPM = wheel1Torque;
    wheel2_refRPM = wheel2Torque;
    wheel3_refRPM = wheel3Torque;
    wheel4_refRPM = wheel4Torque;
}