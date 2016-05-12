#include "BaseKinematics.h"

#define M_PI 3.14159265358979323846

const float wheelRadius = 0.03f; //meter - r:30mm
const float lengthBetweenFrontAndRearWheels = 0.2289f; //meter - a:228.9mm
const float lengthBetweenFrontWheels = 0.214f; //meter - b:214mm
const float geom_factor = (lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0);

//set base velocity
//Calculates from the cartesian velocity the individual wheel velocities 
//@param longitudinalVelocity is the forward or backward velocity
//@param transversalVelocity is the sideway velocity
//@param angularVelocity is the rotational velocity around the center
//@param wheelVelocities are the individual wheel velocities
void cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity, float& W0_RPM, float& W1_RPM, float& W2_RPM, float& W3_RPM){
  float geom_factor  = (lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0);
  
  float W0_angVel = longitudinalVelocity + transversalVelocity + geom_factor*angularVelocity;
  float W1_angVel = longitudinalVelocity - transversalVelocity - geom_factor*angularVelocity;
  float W2_angVel = longitudinalVelocity - transversalVelocity + geom_factor*angularVelocity;
  float W3_angVel = longitudinalVelocity + transversalVelocity - geom_factor*angularVelocity;

  //Angular velocity to RPM
  W0_RPM = W0_angVel*60.0/(2*M_PI*wheelRadius);
  W1_RPM = W1_angVel*60.0/(2*M_PI*wheelRadius);
  W2_RPM = W2_angVel*60.0/(2*M_PI*wheelRadius);
  W3_RPM = W3_angVel*60.0/(2*M_PI*wheelRadius);
}

//get base velocity
//Calculates from the wheel velocities the cartesian velocity
//@param wheelVelocities are the velocities of the individual wheels
//@param longitudinalVelocity is the forward or backward velocity
//@param transversalVelocity is the sideway velocity
//@param angularVelocity is the rotational velocity around the center
void wheelVelocitiesToCartesianVelocity(float W0_RPM, float W1_RPM, float W2_RPM, float W3_RPM, float& longitudinalVelocity, float& transversalVelocity, float& angularVelocity){
  //RPM to rad/s
  float W0_angVel = W0_RPM*2*M_PI/60.0;
  float W1_angVel = W1_RPM*2*M_PI/60.0;
  float W2_angVel = W2_RPM*2*M_PI/60.0;
  float W3_angVel = W3_RPM*2*M_PI/60.0;

  longitudinalVelocity = (W0_angVel + W1_angVel + W2_angVel + W3_angVel)*wheelRadius/4.0;
  transversalVelocity = (W0_angVel - W1_angVel - W2_angVel + W3_angVel)*wheelRadius/4.0;
  angularVelocity = (W0_angVel - W1_angVel + W2_angVel - W3_angVel)*wheelRadius/(4.0*geom_factor);
}

//set base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
//void cartesianPositionToWheelPositions();

//get base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
void wheelPositionsToCartesianPosition(int32_t* encoderPositions, int32_t* lastEncoderPositions, float& longitudinalPosition, float& transversalPosition, float& orientation){
  float geom_factor = (lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0);

  float dPosW0 = (encoderPositions[0] - lastEncoderPositions[0])/334.0;
  float dPosW1 = (encoderPositions[1] - lastEncoderPositions[1])/334.0;
  float dPosW2 = (encoderPositions[2] - lastEncoderPositions[2])/334.0;
  float dPosW3 = (encoderPositions[3] - lastEncoderPositions[3])/334.0;

  lastEncoderPositions[0] = encoderPositions[0];
  lastEncoderPositions[1] = encoderPositions[1];
  lastEncoderPositions[2] = encoderPositions[2];
  lastEncoderPositions[3] = encoderPositions[3];

  float deltaLongitudinalPos = (dPosW0 + dPosW1 + dPosW2 + dPosW3) * 2 * M_PI * wheelRadius/4.0;
  float deltaTransversalPos = (dPosW0 - dPosW1 - dPosW2 + dPosW3) * 2 * M_PI * wheelRadius/4.0;

  //Radians
  orientation += (dPosW0 - dPosW1 + dPosW2 - dPosW3) * (2 * M_PI * wheelRadius / (geom_factor));

  longitudinalPosition += deltaLongitudinalPos * cos(orientation) - deltaTransversalPos * sin(orientation);
  
  transversalPosition += deltaLongitudinalPos * sin(orientation) + deltaTransversalPos * cos(orientation);
}

