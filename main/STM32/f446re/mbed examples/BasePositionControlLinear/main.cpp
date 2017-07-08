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
float KLp = 3000.0f, KLd = 200.0f, KLi = 100.0f;
float KTp = 3000.0f, KTd = 200.0f, KTi = 100.0f;
float KOp = 3000.0f, KOd = 200.0f, KOi = 100.0f;


//******************************************************************************
//  Cubic Polynomial Trajectory Variables
//******************************************************************************

//Trajectory 1
//0.3m - 0.3m - 2*Pi rad
float traj1_consts[3][6] = {  
   {0.0f, 0.0f, 0.0f, 3.0f/15625.0f, -9.0f/781250.0f, 9.0f/48828125.0f},        /*  Longituadinal */
   {0.0f, 0.0f, 0.0f, 3.0f/15625.0f, -9.0f/781250.0f, 9.0f/48828125.0f},        /*  Transversal */
   {0.0f, 0.0f, 0.0f, 314.0f/78125.0f, -471.0f/1953125.0f, 942.0f/244140625.0f} /*  Orientation */
};

void cubicPolynomialTrajectory(){
    //Trajectory 1
    if(currentTime <= 25.0f){
        refLongitudinalPosition = traj1_consts[0][0] + traj1_consts[0][1]*currentTime + traj1_consts[0][2]*powf(currentTime,2) + traj1_consts[0][3]*powf(currentTime,3) + traj1_consts[0][4]*powf(currentTime,4) + traj1_consts[0][5]*powf(currentTime,5);
        refTransversalPosition = traj1_consts[1][0] + traj1_consts[1][1]*currentTime + traj1_consts[1][2]*powf(currentTime,2) + traj1_consts[1][3]*powf(currentTime,3) + traj1_consts[1][4]*powf(currentTime,4) + traj1_consts[1][5]*powf(currentTime,5);
        refOrientation = traj1_consts[2][0] + traj1_consts[2][1]*currentTime + traj1_consts[2][2]*powf(currentTime,2) + traj1_consts[2][3]*powf(currentTime,3) + traj1_consts[2][4]*powf(currentTime,4) + traj1_consts[2][5]*powf(currentTime,5);
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
