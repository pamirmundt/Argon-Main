#include "mbed.h"
#include "base.h"

#include "PID.h"
#include "kinematics.h"

//Define Pi
#define M_PI    3.14159265358979323846f

Ticker velocityControlLoop;
Ticker positionControlLoop;
Ticker cubicTrajectories;

//Trajectory Variable
float currentTime = 0.0f;
float trajectoryUpdatePeriod = 0.1f;

//******************************************************************************
//  Velocity PID Control Variables
//******************************************************************************
//Wheel Velocity Control Frequency at 100Hz
float velocityContPeriod = 0.01f;
volatile float wheel1_refRPM = 0.0f ,wheel2_refRPM = 0.0f, wheel3_refRPM = 0.0f, wheel4_refRPM = 0.0f;
//Velocity PID DConstans
float Imax = 4096.0f, Imin = -4096.0f;
float wheel1_Kp = 0.39976f, wheel1_Ki = 23.7406f;
float wheel2_Kp = 0.39976f, wheel2_Ki = 23.7406f;
float wheel3_Kp = 0.39976f, wheel3_Ki = 23.7406f;
float wheel4_Kp = 0.39976f, wheel4_Ki = 23.7406f;

//******************************************************************************
//Position PID Control Variables
//******************************************************************************
//Base Position Control Frequecy at 10Hz
float positionContPeriod = 0.1f;
volatile float refLongitudinalPosition = 0.0f, refTransversalPosition = 0.0f, refOrientation = 0.0f;
volatile float wheel1Torque = 0.0f, wheel2Torque = 0.0f, wheel3Torque = 0.0f, wheel4Torque = 0.0f;
volatile float longitudinalPosition = 0.0f, transversalPosition = 0.0f, orientation = 0.0f;
//Position PID DConstans
float KLp = 200.0f, KLd = 0.0f, KLi = 1000.0f;
float KTp = 200.0f, KTd = 0.0f, KTi = 1000.0f;
float KOp = 200.0f, KOd = 0.0f, KOi = 1000.0f;


//******************************************************************************
//  Cubic Polynomial Trajectory Variables
//******************************************************************************

void cubicPolynomialTrajectory(){
    float Ctrans = -0.148f/2.0f;
    float Clong = 0.160f/2.0f;
    
    float r = sqrtf((powf(Clong,2) + pow(Ctrans,2)));
    
    refLongitudinalPosition = Clong + r*sin(currentTime+atan(Clong/Ctrans));
    
    refTransversalPosition = Ctrans + r*cos(currentTime+atan(Clong/Ctrans));
    
    refOrientation = -currentTime;
    
    currentTime += trajectoryUpdatePeriod/4.0f;
}

int main() {
    //Init Argon Base
    baseInit();
    baseEncoderStart();
    baseRpmStart();
    basePwmStart();
    
    //Set Base Velocity
    cartesianVelocityToWheelVelocities(0.0f, 0.0f, 0.0f);
    
    //Start Interrupt at 100Hz for velocity PID Control
    velocityControlLoop.attach(&calcVelocityPID, velocityContPeriod);
    
    //Start Interrupt at 10Hz for position PID Control
    positionControlLoop.attach(&calcPositionPID, positionContPeriod);
    
    //Trajectory
    cubicTrajectories.attach(&cubicPolynomialTrajectory, trajectoryUpdatePeriod);
    
    while(1) {
        //Update Base Position
        wheelPositionsToCartesianPosition();
    }
}
