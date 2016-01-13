#define encoderRes 334                    //Encoder Resolution 344 pulse/rotation
#define gearRatio 9.28f                   //Gear Ratio 1:9.28
#define contPeriod 10000.0f               //Control Period - 100Hz to microsecond  1/100sec = 0.01sec
#define contPerSec contPeriod/1000000.0f  //microsecond to second
#define intRes 3

//PID Parameters
//Motor-1
const float Kp1 = 0.3;
const float Ki1 = 0.8;
const float Kd1 = 0.0;
