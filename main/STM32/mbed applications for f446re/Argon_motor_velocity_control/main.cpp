#include "mbed.h"
#include "base.h"

#include "PID.h"

#define M_PI 3.14159265358979323846f

//Interrupt for PID Loop and Sin Wave
Ticker controlLoop;
Ticker generateSine;

//Time
float t = 0.0f;

//Control Frequency 100Hz
float contPeriod = 0.01f;

//Integral Windup Limit
float Imax = 4096.0f, Imin = -4096.0f;

//PID Constants
float wheel1_Kp = 0.39976f, wheel1_Ki = 23.7406f;
float wheel2_Kp = 0.39976f, wheel2_Ki = 23.7406f;
float wheel3_Kp = 0.39976f, wheel3_Ki = 23.7406f;
float wheel4_Kp = 0.39976f, wheel4_Ki = 23.7406f;

//Reference Wheel RPMs
float wheel1_refRPM = 0.0f, wheel2_refRPM = 0.0f, wheel3_refRPM = 0.0f, wheel4_refRPM = 0.0f;


/* Sine Wave Example ---------------------------------------------------------*/

void sineWave(){
    wheel1_refRPM = 40.0f*sin(t*M_PI/180.0f);
    wheel2_refRPM = 40.0f*sin(t*M_PI/180.0f);
    wheel3_refRPM = 40.0f*sin(t*M_PI/180.0f);
    wheel4_refRPM = 40.0f*sin(t*M_PI/180.0f);
    
    t += 2.0f;
}

/* Main ----------------------------------------------------------------------*/

int main()
{
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();

    //Start PID Control Loop
    controlLoop.attach(&calcPID, contPeriod);
    
    //Start Sine Wave Reference
    generateSine.attach(&sineWave, 0.01);
    
    while(1) {
        //----->
    }
}
