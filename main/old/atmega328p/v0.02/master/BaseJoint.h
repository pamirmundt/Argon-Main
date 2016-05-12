//Arduino
//Base joint library

#ifndef BASEJOINT_H
#define BASEJOINT_H

#include <stdint.h>
#include <Wire.h>
#include <Arduino.h>  //delay() function

enum{
  MANUEL_MODE,
  POSITION_MODE,
  VELOCITY_MODE
};

class Wheel{
  public:
    //Wheel default constructor
    Wheel();

    //Wheel constructor
    //@param wheelNumber (0-3)
    //@param slaveAddress
    Wheel(uint16_t wheelNum, uint16_t slaveAddr);

    //Set Commands
    //Resets the motor variables and sets PID constants to default value
    void reset();

    //Sets the Reference RPM of the wheel
    //Needs to be switched to velocity control mode(2) before sending command
    //@param Reference RPM 
    void moveSpeed(float RPM);

    //Sets the Reference Position of the wheel (as encoder ticks)
    //Needs to be switched to position control mode(1) before sending command
    //@param Reference Position as encoder ticks
    void moveTo(int32_t refPos);

    //Sets the PID controller constants of the wheel
    //@param proportionalConstant
    //@param integralConstant
    //@param derivativeConstant
    void setPID(float Kp, float Ki, float Kd);

    //Sets the power of the wheel (0-100%)
    //Needs to be switched to manuel mode(0) before sending command
    //@param power
    void setPower(float power);

    //Sets the PWM of the wheel (9bit resolution, 0-511)
    //Needs to be switched to manuel mode(0) before sending command
    //@param PWM
    void setPWM(float PWM);

    //Sets the control mode of the wheel
    //0 - Manuel Mode (setPower, setPWM, etc.)
    //1 - Position Control Mode with PID
    //2 - Velocity Control Mode with PID
    //@param driveMode
    void setMode(uint8_t driveMode);

    //Get Commands
    //Get the current PID constrants of the wheel
    //@param proportionalConstant
    //@param integralConstant
    //@param derivativeConstant
    void getPID(float* Kp, float* Ki, float* Kd);

    //Returns the current power (0-100%)
    float getPower();

    //Returns the position as encoder Ticks
    int32_t getPos();

    //Returns the velocity as RPM
    float getSpeed();

    //Returns the PWM (0-511)
    float getPWM();

    //Returns the reference speed as RPM
    float getRefSpeed();

    //Returns the reference position as encoder ticks
    int32_t getRefPos();

    //Returns the I2C address
    uint16_t getAddr();

    //Returns the number of the wheel (0-3)
    uint16_t getWheelNum();
    
  private:
    uint16_t slaveAddr;
    uint16_t wheelNum;
    
    bool lastWheelPositionInitialized;
    int32_t lastEncPos;
    /*
    uint8_t driveMode;

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

#endif