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

//Trajectory 1
//0.3m - 0.3m - 2*Pi rad
float traj1_consts[3][6] = {  
   {0.0f, 0.0f, 0.0f, 1.0f/1125.0f, -1.0f/11250.0f, 1.0f/421875.0f},            /*  Longituadinal */
   {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                                        /*  Transversal */
   {0.0f, 0.0f, 0.0f, -M_PI/675.0f, M_PI/6750.0f, -M_PI/253125.0f}              /*  Orientation */
};

float traj2_consts[3][6] = {  
   {3.0f/10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                                  /*  Longituadinal */
   {-93.0f/10.0f, 12.0f/5.0f, -6.0f/25.0f, 13.0f/1125.0f, -1.0f/3750.0f, 1.0f/421875},  /*  Transversal */
   {15.0f*M_PI, -4.0f*M_PI, (2*M_PI)/5.0f, -(13.0f*M_PI)/675.0f, M_PI/2250.0f, -M_PI/253125.0f} /*  Orientation */
};

float traj3_consts[3][6] = {  
   {1539.0f/10.0f, -108.0f/5.0f, 6.0f/5.0f, -37.0f/1125.0f, 1.0f/2250.0f, -1.0f/421875.0f},        /*  Longituadinal */
   {3.0f/10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},        /*  Transversal */
   {255.0f*M_PI, -36.0f*M_PI, 2.0f*M_PI, -(37.0f*M_PI)/675.0f, M_PI/1350.0f, -M_PI/253125.0f} /*  Orientation */
};

float traj4_consts[3][6] = {  
   {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},        /*  Longituadinal */
   {4416.0f/5.0f, -432.0f/5.0f, 84.0f/25.0f, -73.0f/1125.0f, 7.0f/11250.0f, -1.0f/421875.0f},        /*  Transversal */
   {1470.0f*M_PI, -144.0f*M_PI, 28.0f*M_PI/5.0f, -(73.0f*M_PI)/675.0f, 7.0f*M_PI/6750.0f, -M_PI/253125.0f} /*  Orientation */
};


void cubicPolynomialTrajectory(){
    //Trajectory 1
    if(currentTime <= 15.0f){
        refLongitudinalPosition = traj1_consts[0][0] + traj1_consts[0][1]*currentTime + traj1_consts[0][2]*powf(currentTime,2) + traj1_consts[0][3]*powf(currentTime,3) + traj1_consts[0][4]*powf(currentTime,4) + traj1_consts[0][5]*powf(currentTime,5);
        refTransversalPosition = traj1_consts[1][0] + traj1_consts[1][1]*currentTime + traj1_consts[1][2]*powf(currentTime,2) + traj1_consts[1][3]*powf(currentTime,3) + traj1_consts[1][4]*powf(currentTime,4) + traj1_consts[1][5]*powf(currentTime,5);
        refOrientation = traj1_consts[2][0] + traj1_consts[2][1]*currentTime + traj1_consts[2][2]*powf(currentTime,2) + traj1_consts[2][3]*powf(currentTime,3) + traj1_consts[2][4]*powf(currentTime,4) + traj1_consts[2][5]*powf(currentTime,5);
    }
    if(currentTime >= 15.0f && currentTime <= 30.0f){
        refLongitudinalPosition = traj2_consts[0][0] + traj2_consts[0][1]*currentTime + traj2_consts[0][2]*powf(currentTime,2) + traj2_consts[0][3]*powf(currentTime,3) + traj2_consts[0][4]*powf(currentTime,4) + traj2_consts[0][5]*powf(currentTime,5);
        refTransversalPosition = traj2_consts[1][0] + traj2_consts[1][1]*currentTime + traj2_consts[1][2]*powf(currentTime,2) + traj2_consts[1][3]*powf(currentTime,3) + traj2_consts[1][4]*powf(currentTime,4) + traj2_consts[1][5]*powf(currentTime,5);
        refOrientation = traj2_consts[2][0] + traj2_consts[2][1]*currentTime + traj2_consts[2][2]*powf(currentTime,2) + traj2_consts[2][3]*powf(currentTime,3) + traj2_consts[2][4]*powf(currentTime,4) + traj2_consts[2][5]*powf(currentTime,5);
    }
    if(currentTime >= 30.0f && currentTime <= 45.0f){
        refLongitudinalPosition = traj3_consts[0][0] + traj3_consts[0][1]*currentTime + traj3_consts[0][2]*powf(currentTime,2) + traj3_consts[0][3]*powf(currentTime,3) + traj3_consts[0][4]*powf(currentTime,4) + traj3_consts[0][5]*powf(currentTime,5);
        refTransversalPosition = traj3_consts[1][0] + traj3_consts[1][1]*currentTime + traj3_consts[1][2]*powf(currentTime,2) + traj3_consts[1][3]*powf(currentTime,3) + traj3_consts[1][4]*powf(currentTime,4) + traj3_consts[1][5]*powf(currentTime,5);
        refOrientation = traj3_consts[2][0] + traj3_consts[2][1]*currentTime + traj3_consts[2][2]*powf(currentTime,2) + traj3_consts[2][3]*powf(currentTime,3) + traj3_consts[2][4]*powf(currentTime,4) + traj3_consts[2][5]*powf(currentTime,5);
    }
    
    if(currentTime >= 45.0f && currentTime <= 60.0f){
        refLongitudinalPosition = traj4_consts[0][0] + traj4_consts[0][1]*currentTime + traj4_consts[0][2]*powf(currentTime,2) + traj4_consts[0][3]*powf(currentTime,3) + traj4_consts[0][4]*powf(currentTime,4) + traj4_consts[0][5]*powf(currentTime,5);
        refTransversalPosition = traj4_consts[1][0] + traj4_consts[1][1]*currentTime + traj4_consts[1][2]*powf(currentTime,2) + traj4_consts[1][3]*powf(currentTime,3) + traj4_consts[1][4]*powf(currentTime,4) + traj4_consts[1][5]*powf(currentTime,5);
        refOrientation = traj4_consts[2][0] + traj4_consts[2][1]*currentTime + traj4_consts[2][2]*powf(currentTime,2) + traj4_consts[2][3]*powf(currentTime,3) + traj4_consts[2][4]*powf(currentTime,4) + traj4_consts[2][5]*powf(currentTime,5);
    }
    
    currentTime += trajectoryUpdatePeriod;
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
