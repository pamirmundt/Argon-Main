

extern float wheel1_refRPM, wheel2_refRPM , wheel3_refRPM, wheel4_refRPM;

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