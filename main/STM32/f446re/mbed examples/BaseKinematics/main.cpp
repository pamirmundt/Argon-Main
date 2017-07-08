#include "mbed.h"
#include "base.h"

#include "PID.h"
#include "kinematics.h"

/* Main ---------------------------------------------------*/

int main() {
    //Initialize Base
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();

    //Base Velocity to Wheel Velocities
    cartesianVelocityToWheelVelocities(float longitudinalVelocity, float transversalVelocity, float angularVelocity);
    
    //Wheel Velocities to Base Velocity
    wheelVelocitiesToCartesianVelocity(float W1_RPM, float W2_RPM, float W3_RPM, float W4_RPM, float* longitudinalVelocity, float* transversalVelocity, float* angularVelocity);
    
    //Wheel Position to Base Position
        //- Needs to be computed in every loop
    wheelPositionsToCartesianPosition();
    
    //Calculate Jacobian Transpose
    calcJacobianT(float baseLongitudinalForce, float baseTransversalForce, float baseOrientationForce);
        
    while(1) {
        
        //----->
        
    }
}
