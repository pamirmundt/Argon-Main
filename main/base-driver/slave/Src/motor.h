#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"

typedef struct{
	volatile uint8_t mode;
  
  volatile float PWM;
  volatile float RPM;
  volatile int32_t encPos;
  volatile int32_t encPrevPos;
  volatile int32_t dPos;

  //PID
  volatile float Kp;
  volatile float Ki;
  volatile float Kd;
  volatile float integral;
  volatile float derivative;
  volatile float control;
  volatile float errRPM;
  volatile float errPrevRPM;
  volatile float refRPM;
}Motor;

#endif
