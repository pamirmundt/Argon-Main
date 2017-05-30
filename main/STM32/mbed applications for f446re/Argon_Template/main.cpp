/**
  ******************************************************************************
  * Template for Argon
  ******************************************************************************
**/

#include "mbed.h"
#include "base.h"

//Serial pc(USBTX, USBRX);

int main() {
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();
    
    setMotorDirection(frontLeft,forward);
    setMotorDirection(frontRight,backward);
    setMotorDirection(rearLeft,forward);
    setMotorDirection(rearRight,backward);
    
    setMotorPWM(frontLeft,256);
    setMotorPWM(frontRight,256);
    setMotorPWM(rearLeft,256);
    setMotorPWM(rearRight,256);
    
    while(1) {                 
        //----->
    }
}
