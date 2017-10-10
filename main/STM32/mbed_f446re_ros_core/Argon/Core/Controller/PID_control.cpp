#include "PID_control.h"


void velocityPID::start(void){
    this->controlLoop.attach(this,&velocityPID::calcPID, this->contPeriod);
}

void velocityPID::setRef(float _reference){ 
    this->reference = _reference;
}

float velocityPID::getRef(void){
    return reference;   
}

void velocityPID::calcPID(void){
    float errRPM = this->reference - this->motor.getJointRPM();
    this->integralVal = this->integralVal + errRPM * this->contPeriod;
    
    float controlVal = this->Kp*errRPM + this->Kd*integralVal;
    
    this->motor.setPWM(controlVal);
}