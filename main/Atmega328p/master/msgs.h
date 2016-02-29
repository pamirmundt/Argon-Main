#ifndef MSGS_H
#define MSHS_H

#define MSGS_FREQ 10.0f //Hz

typedef enum{
  FRONT_LEFT_WHEEL,
  FRONT_RIGHT_WHEEL,
  REAR_LEFT_WHEEL,
  REAR_RIGHT_WHEEL
};

typedef struct{
  uint16_t wheel;
  int32_t encPos;
  float RPM;
  float PWM;
  int32_t refPos;
  float refRPM;
  float controlFreq;
  float msgsFreq;
}MSGS;

#endif
