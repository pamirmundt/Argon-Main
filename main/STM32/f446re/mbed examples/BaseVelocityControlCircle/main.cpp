#include "mbed.h"
#include "Argon/base.h"

#include "PID.h"
#include "kinematics.h"

//Define Pi
#define M_PI    3.14159265358979323846f

//Interrupts
Ticker controlLoop;
Ticker sineWaveGenerate;

//Time
float i = 0.0f;

//Control Frequency 100Hz
float contPeriod = 0.01f;

//PI Constans
float Imax = 4096.0f, Imin = -4096.0f;
float wheel1_Kp = 0.39976f, wheel1_Ki = 23.7406f;
float wheel2_Kp = 0.39976f, wheel2_Ki = 23.7406f;
float wheel3_Kp = 0.39976f, wheel3_Ki = 23.7406f;
float wheel4_Kp = 0.39976f, wheel4_Ki = 23.7406f;

//Reference RPMs
float wheel1_refRPM = 0.0f ,wheel2_refRPM = 0.0f, wheel3_refRPM = 0.0f, wheel4_refRPM = 0.0f;


/* Sine Wave - draw circle ---------------------------------------------------*/
void sineWave(){
    float reflongVel = 0.05f*sin(i*M_PI/180.0f - M_PI/2.0f);
    float refTransVel = 0.05f*cos(i*M_PI/180.0f - M_PI/2.0f);
    float refOrienVel = 0.0f;
    
    cartesianVelocityToWheelVelocities(reflongVel, refTransVel, refOrienVel);
    
    i += 1.0f;
}

/* Main ---------------------------------------------------*/

int main() {
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();
    
    //Start Interrupt at 100Hz
    controlLoop.attach(&calcPID, contPeriod);
    
    //float longitudinalVelocity, transversalVelocity, angularVelocity;
    sineWaveGenerate.attach(sineWave, 0.01);
        
    while(1) {
        //----->
    }
}
