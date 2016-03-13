//Arduino
//Base library without Kinematics

#ifndef BASE
#define BASE

//BASE -> 4 motors ->

enum{
  MANUEL_MODE,
  POSITION_MODE,
  VELOCITY_MODE
};

class Wheel{
  public:
    Wheel();
    Wheel(uint16_t, uint16_t);

    //Set Commands
    void reset();
    void moveSpeed(float);
    void moveTo(int32_t);
    void setPID(float, float, float);
    void setPower(float);
    void setPWM(float);
    void setMode(uint8_t);

    //Get Commands
    void getPID(float*, float*, float*);
    float getPower();
    int32_t getPos();
    float getSpeed();
    float getPWM();
    float getRefSpeed();
    uint16_t getAddr();
    uint16_t getWheelNum();
    int32_t getRefPos();
    
  private:
    uint16_t slaveAddr;
    uint16_t wheelNum;
    uint8_t driveMode;

    /*
    int32_t encPos;
    float RPM;
    float PWM;
    int32_t refPos;
    float refRPM;
    float controlFreq;
    float msgsFreq;
    float Kp;
    float Ki;
    float Kd;
    float power;
    */
};

/*
class Base{
  public:
    Wheel wheel_0;
    Wheel wheel_1;
    Wheel wheel_2;
    Wheel wheel_3;
};
*/

#endif
