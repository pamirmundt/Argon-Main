#include "baseKinematics.h"
#include <math.h>

#define encChannelRes                     4.0f    //4x Encoding - Channal A/B
#define encoderRes                        334.0f	//Encoder Resolution 334 pulse/rotation
#define wheelRadius                       0.03f   //meter - r:30mm
//#define gearRatio                         13.552f //1:13.552 Ratio
#define lengthBetweenFrontAndRearWheels   0.2289f //meter - a:228.9mm
#define lengthBetweenFrontWheels          0.214f  //meter - b:214mm
#define geom_factor                       (lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0f)

void cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity, float* W0_RPM, float* W1_RPM, float* W2_RPM, float* W3_RPM){
 
  float W0_angVel = longitudinalVelocity + transversalVelocity + geom_factor*angularVelocity;
  float W1_angVel = longitudinalVelocity - transversalVelocity - geom_factor*angularVelocity;
  float W2_angVel = longitudinalVelocity - transversalVelocity + geom_factor*angularVelocity;
  float W3_angVel = longitudinalVelocity + transversalVelocity - geom_factor*angularVelocity;

  //Angular velocity to RPM
  *W0_RPM = W0_angVel * 60.0f / (2.0f * M_PI * wheelRadius);
  *W1_RPM = W1_angVel * 60.0f / (2.0f * M_PI * wheelRadius);
  *W2_RPM = W2_angVel * 60.0f / (2.0f * M_PI * wheelRadius);
  *W3_RPM = W3_angVel * 60.0f / (2.0f * M_PI * wheelRadius);
}

//get base velocity
//Calculates from the wheel velocities the cartesian velocity
//@param wheelVelocities are the velocities of the individual wheels
//@param longitudinalVelocity is the forward or backward velocity
//@param transversalVelocity is the sideway velocity
//@param angularVelocity is the rotational velocity around the center
void wheelVelocitiesToCartesianVelocity(float W0_RPM, float W1_RPM, float W2_RPM, float W3_RPM, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity){
  //RPM to rad/s
  float W0_angVel = W0_RPM * 2.0f * M_PI / 60.0f;
  float W1_angVel = W1_RPM * 2.0f * M_PI / 60.0f;
  float W2_angVel = W2_RPM * 2.0f * M_PI / 60.0f;
  float W3_angVel = W3_RPM * 2.0f * M_PI / 60.0f;

  *longitudinalVelocity = (W0_angVel + W1_angVel + W2_angVel + W3_angVel) * wheelRadius / 4.0f;
  *transversalVelocity = (W0_angVel - W1_angVel - W2_angVel + W3_angVel) * wheelRadius /4.0f;
  *angularVelocity = (W0_angVel - W1_angVel + W2_angVel - W3_angVel) * wheelRadius /(4.0f*geom_factor);
}

//get base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
void wheelPositionsToCartesianPosition(int32_t encoderPositions[], int32_t lastEncoderPositions[], float* longitudinalPosition, float* transversalPosition, float* orientation){
  float dPosW0 = ((float)(encoderPositions[0] - lastEncoderPositions[0]));
  float dPosW1 = ((float)(encoderPositions[1] - lastEncoderPositions[1]));
  float dPosW2 = ((float)(encoderPositions[2] - lastEncoderPositions[2]));
  float dPosW3 = ((float)(encoderPositions[3] - lastEncoderPositions[3]));

  lastEncoderPositions[0] = encoderPositions[0];
  lastEncoderPositions[1] = encoderPositions[1];
  lastEncoderPositions[2] = encoderPositions[2];
  lastEncoderPositions[3] = encoderPositions[3];

  float deltaLongitudinalPos = (dPosW0 + dPosW1 + dPosW2 + dPosW3) * 2.0f * M_PI * wheelRadius / 4.0f / encoderRes / gearRatio / encChannelRes;
  float deltaTransversalPos = (dPosW0 - dPosW1 - dPosW2 + dPosW3) * 2.0f * M_PI * wheelRadius / 4.0f / encoderRes / gearRatio / encChannelRes;

  //Radians
  *orientation += (dPosW0 - dPosW1 + dPosW2 - dPosW3) * 2.0f * M_PI * wheelRadius / geom_factor / encoderRes / gearRatio / encChannelRes;

  *longitudinalPosition += deltaLongitudinalPos * cos(*orientation) - deltaTransversalPos * sin(*orientation);

  *transversalPosition += deltaLongitudinalPos * sin(*orientation) + deltaTransversalPos * cos(*orientation);
}

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base longitudinal force
//@param base transversal force
//@param base orientation force
//@param returns wheel torques
void calcJacobianT(volatile float* baseLongitudinalForce, volatile float* baseTransversalForce, volatile float* baseOrientationForce, volatile float wheelTorques[]){
	wheelTorques[0] = (*baseLongitudinalForce + *baseTransversalForce + *baseOrientationForce * (1.0f / geom_factor)) * (wheelRadius/4.0f);
	wheelTorques[1] = (*baseLongitudinalForce - *baseTransversalForce - *baseOrientationForce * (1.0f / geom_factor)) * (wheelRadius/4.0f);
	wheelTorques[2] = (*baseLongitudinalForce - *baseTransversalForce + *baseOrientationForce * (1.0f / geom_factor)) * (wheelRadius/4.0f);
	wheelTorques[3] = (*baseLongitudinalForce + *baseTransversalForce - *baseOrientationForce * (1.0f / geom_factor)) * (wheelRadius/4.0f);
}
