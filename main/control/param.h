#define encoderRes 344    //Encoder Resolution 344 pulse/rotation
#define gearRatio 9.28    //Gear Ratio 1:9.28
#define contPer 2000.0   //Control Period - 500Hz to microsecond  1/500sec = 0.002sec

//PID Parameters
//Motor-1
const double Kp1 = 0.00001;
const double Ki1 = 0.000001;
const double Kd1 = 0.0;
