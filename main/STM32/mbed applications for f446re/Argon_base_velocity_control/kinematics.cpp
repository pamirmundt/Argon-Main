#include "mbed.h"
#include "base.h"

#include "kinematics.h"

#define M_PI    3.14159265358979323846f

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