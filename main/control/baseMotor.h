#ifndef BASE_MOTOR_H
#define BASE_MOTOR_H

//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 9.28

//Motor and controller parameters
#define encoderRes 334                    //Encoder Resolution 344 pulse/rotation
#define gearRatio 9.28f                   //Gear Ratio 1:9.28
#define contPerSec 0.01f                  //Control Period - 100Hz to microsecond  1/100sec = 0.01sec

const float Kp = 0.3;
const float Ki = 0.8;
const float Kd = 0.0;

enum{
  FRONT_LEFT_WHEEL,
  FRONT_RIGHT_WHEEL,
  REAR_LEFT_WHEEL,
  REAR_RIGHT_WHEEL
};

typedef struct{
  int8_t index;
  
  volatile int32_t refRPM;
  volatile int32_t RPM;
  volatile int32_t pos;
  volatile int32_t posPrev;
  volatile int32_t encPosA;
  volatile int32_t encPosB;
  volatile int32_t dPos;

  float Kp;
  float Ki;
  float Kd;
  volatile int32_t errRPM;
  volatile int32_t integral;
  volatile int32_t derivative;
  volatile int32_t errPrevRPM;
  volatile int32_t control;
}BMotor;

#endif
