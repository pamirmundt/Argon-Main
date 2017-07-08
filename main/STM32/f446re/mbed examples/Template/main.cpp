/**
  ******************************************************************************
  * Template for Argon
  ******************************************************************************
**/

#include "mbed.h"
#include "Argon/base.h"

int main() {
    //Initialize Base
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();
    
    //Set Motor Directions
    setMotorDirection(frontLeft, forward);
    setMotorDirection(frontRight, forward);
    setMotorDirection(rearLeft, forward);
    setMotorDirection(rearRight, forward);
    
    //Set Motor PWMs
    setMotorPWM(frontLeft, 0);
    setMotorPWM(frontRight, 0);
    setMotorPWM(rearLeft, 0);
    setMotorPWM(rearRight, 0);
    
    while(1) {
        
        //----->
        
    }
}
