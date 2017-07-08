
extern volatile float wheel1_refRPM, wheel2_refRPM, wheel3_refRPM, wheel4_refRPM;
extern volatile float longitudinalPosition, transversalPosition, orientation;
extern volatile float wheel1Torque, wheel2Torque, wheel3Torque, wheel4Torque;


//Calculates from the cartesian velocity the individual wheel velocities 
//@param - longitudinalVelocity is the forward or backward velocity (m/s)
//@param - transversalVelocity is the sideway velocity (m/s)
//@param - angularVelocity is the rotational velocity around the center (rad/s)
//@param - wheelVelocities are the individual wheel velocities (RPM)
void cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity);

//Calculates from the wheel velocities the cartesian velocity
//@param wheelVelocities are the velocities of the individual wheels (RPM)
//@param longitudinalVelocity is the forward or backward velocity (m/s)
//@param transversalVelocity is the sideway velocity (m/s)
//@param angularVelocity is the rotational velocity around the center (rad/s)
void wheelVelocitiesToCartesianVelocity(float W1_RPM, float W2_RPM, float W3_RPM, float W4_RPM, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity);

//Update/get base position
//Calculates from the cartesian position the wheel positions
//@param longitudinalPosition is the forward or backward position
//@param transversalPosition is the sideway position
//@param orientation is the rotation around the center
//@param wheelPositions are the individual positions of the wheels
void wheelPositionsToCartesianPosition();

//calculate wheel torques
//Calculates wheel torques from base force with Jacobian Transpose
//@param base longitudinal force
//@param base transversal force
//@param base orientation force
//@param returns wheel torques
void calcJacobianT(float baseLongitudinalForce, float baseTransversalForce, float baseOrientationForce);