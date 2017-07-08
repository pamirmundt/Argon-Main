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
    setMotorDirection(frontRight, backward);
    setMotorDirection(rearLeft, forward);
    setMotorDirection(rearRight, backward);
    
    //Set Motor PWMs
    setMotorPWM(frontLeft, 256);
    setMotorPWM(frontRight, 256);
    setMotorPWM(rearLeft, 256);
    setMotorPWM(rearRight, 256);
    
    while(1) {
        
        //----->
        
    }
}
