#ifndef BASE_MOTOR_H
#define BASE_MOTOR_H

//Motor: JGA25-271
//Encoder Resolution: 344 pulse/rotation
//Gear Ratio: 13.552

//Motor and controller parameters
#define encoderRes 334                    //Encoder Resolution 344 pulse/rotation
#define gearRatio 13.552f                 //Gear Ratio 1:13.552

//Default Kp, Ki, Kd
float Kp = 0.3;
float Ki = 0.8;
float Kd = 0.0;

typedef enum{
  FRONT_LEFT_WHEEL,
  FRONT_RIGHT_WHEEL,
  REAR_LEFT_WHEEL,
  REAR_RIGHT_WHEEL
};

typedef struct{
  uint16_t wheel;
  
  volatile float PWM;
  volatile float RPM;
  volatile int32_t posEnc;
  volatile int32_t posEncPrev;
  volatile int32_t dPos;

  //PID
  float Kp;
  float Ki;
  float Kd;
  volatile float integral;
  volatile float derivative;
  volatile float control;
  //Velocity PID  
  volatile float errRPM;
  volatile float errPrevRPM;
  volatile float refRPM;
  //Position PID
  volatile int32_t refPos;
  volatile int32_t errPos;
  volatile int32_t errPrevPos;
}Motor;

#endif
