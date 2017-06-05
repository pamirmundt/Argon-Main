/* Includes ------------------------------------------------------------------*/
#include "mbed.h"

#include "PID.h"
#include "base.h"
#include "params.h"
#include "kinematics.h"


/* Local Variables ------------------------------------------------------------*/
//Motor velocity control local variables
volatile float wheel1_integral = 0.0f, wheel2_integral = 0.0f, wheel3_integral = 0.0f, wheel4_integral = 0.0f;

//Base position control local variables
volatile float errPrevLong = 0.0f, errPrevTrans = 0.0f, errPrevOrien = 0.0f;
volatile float integralLong = 0.0f, integralTrans = 0.0f, integralOrien = 0.0f;


/* Functions -----------------------------------------------------------------*/

//Wheels PID control loop
void calcVelocityPID()
{
    /* Wheel 1 Discrete PI Control------------------------------------------- */
    float wheel1_errRPM = wheel1_refRPM*gearRatio - getMotorRPM(1);
    wheel1_integral = wheel1_integral + wheel1_errRPM * velocityContPeriod;
    
    //Integral Windup Check
    if (wheel1_integral > Imax)
        wheel1_integral = Imax;
    else if (wheel1_integral < Imin)
        wheel1_integral = Imin;

    float wheel1_control = wheel1_Kp*wheel1_errRPM + wheel1_Ki*wheel1_integral;
    
    /* Wheel 2 Discrete PI Control------------------------------------------- */
    float wheel2_errRPM = wheel2_refRPM * gearRatio - getMotorRPM(2);
    wheel2_integral = wheel2_integral + wheel2_errRPM * velocityContPeriod;
    
    //Integral Windup Check
    if (wheel2_integral > Imax)
        wheel2_integral = Imax;
    else if (wheel2_integral < Imin)
        wheel2_integral = Imin;
        
    float wheel2_control = wheel2_Kp*wheel2_errRPM + wheel2_Ki*wheel2_integral;
    
    /* Wheel 3 Discrete PI Control------------------------------------------- */
    float wheel3_errRPM = wheel3_refRPM * gearRatio - getMotorRPM(3);
    wheel3_integral = wheel3_integral + wheel3_errRPM * velocityContPeriod;
    
    //Integral Windup Check
    if (wheel3_integral > Imax)
        wheel3_integral = Imax;
    else if (wheel3_integral < Imin)
        wheel3_integral = Imin;
    
    float wheel3_control = wheel3_Kp*wheel3_errRPM + wheel3_Ki*wheel3_integral;
    
    /* Wheel 4 Discrete PI Control------------------------------------------- */
    float wheel4_errRPM = wheel4_refRPM * gearRatio - getMotorRPM(4);
    wheel4_integral = wheel4_integral + wheel4_errRPM * velocityContPeriod;
    
    //Integral Windup Check
    if (wheel4_integral > Imax)
        wheel4_integral = Imax;
    else if (wheel4_integral < Imin)
        wheel4_integral = Imin;
    
    float wheel4_control = wheel4_Kp*wheel4_errRPM + wheel4_Ki*wheel4_integral;

    /* ---------------------------------------------------------------------- */

    //Wheel 1 Direction Control
    if(wheel1_control > 0)
        setMotorDirection(frontLeft, forward);
    else
        setMotorDirection(frontLeft, backward);

    //Wheel 2 Direction Control
    if(wheel2_control > 0)
        setMotorDirection(frontRight, forward);
    else
        setMotorDirection(frontRight, backward);

    //Wheel 3 Direction Control
    if(wheel3_control > 0)
        setMotorDirection(rearLeft, forward);
    else
        setMotorDirection(rearLeft, backward);

    //Wheel 4 Direction Control
    if(wheel4_control > 0)
        setMotorDirection(rearRight, forward);
    else
        setMotorDirection(rearRight, backward);
        
    /* ---------------------------------------------------------------------- */

    //Wheel 1 Set PWM
    setMotorPWM(frontLeft, abs(wheel1_control));
    //Wheel 2 Set PWM
    setMotorPWM(frontRight, abs(wheel2_control));
    //Wheel 3 Set PWM
    setMotorPWM(rearLeft, abs(wheel3_control));
    //Wheel 4 Set PWM
    setMotorPWM(rearRight, abs(wheel4_control));
}

void calcPositionPID(){
    //Base Longitudinal Position PD
    float errLong = refLongitudinalPosition - longitudinalPosition;
    integralLong = integralLong + errLong * positionContPeriod;
    float derivativeLong = (errLong - errPrevLong) / positionContPeriod;
    float controlLongForce = KLp * errLong + KLd * derivativeLong+ KLi * integralLong;
    errPrevLong = errLong;
    
    //Base Transversal Position PD
    float errTrans = refTransversalPosition - transversalPosition;
    integralTrans = integralTrans + errTrans * positionContPeriod;
    float derivativeTrans = (errTrans - errPrevTrans) / positionContPeriod;
    float controlTransForce = KTp * errPrevTrans + KTd * derivativeTrans + KTi * integralTrans;
    errPrevTrans = errTrans;
    
    //Base Orientation PD
    float errOrien = refOrientation - orientation;
    integralOrien = integralOrien + errOrien * positionContPeriod;
    float derivativeOrien = (errOrien - errPrevOrien) / positionContPeriod;
    float controlOrienForce = KOp * errOrien + KOd * derivativeOrien + KOi * integralOrien;
    errPrevOrien = errOrien;

    //Force to Torque
    calcJacobianT(controlLongForce, controlTransForce, controlOrienForce);
}