#ifndef ARGON_MOTOR_DRIVER_H_
#define ARGON_MOTOR_DRIVER_H_

#include "mbed.h"
#include "Motor.h"

class velocityPID{
    private:
        Ticker controlLoop;
        Motor motor;
        float Kp;
        float Ki;
        float Kd;
        float contPeriod;
        void calcPID(void);
        float integralVal;
        float reference;
    public:
        velocityPID(Motor _motor, float _Kp, float _Ki,float _Kd, float _contPeriod):
            Kp(0.0f),Ki(0.0f),Kd(0.0f),contPeriod(0.0f),integralVal(0.0f),reference(0.0f)
        {
            this->motor = _motor;
            this->Kp = _Kp;
            this->Ki = _Ki;
            this->Kd = _Kd;
            contPeriod = _contPeriod;
        }
        void start(void);
        void stop(void);
        void setRef(float);
        float getRef(void);
};

#endif // ARGON_MOTOR_DRIVER_H_