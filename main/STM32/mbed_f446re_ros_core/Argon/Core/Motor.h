#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

enum directions {
    forward,
    backward,
};

enum mecanumWheels {
    noWheel,
    frontLeft,      //WheelNumber 1
    frontRight,     //WheelNumber 2
    rearLeft,       //WheelNumber 3
    rearRight,      //WheelNumber 4
};

class Motor{
    private:
        uint8_t motorNumber;
        //PWM
        TIM_HandleTypeDef htimerPWM;
        uint32_t htimerChannelPWM;
        //Dir
        uint8_t defaultDirection;
        GPIO_TypeDef * directionGpioPort;
        uint16_t directionGpioPin;
        //Encoder
        TIM_HandleTypeDef htimerEncoder;  
        
        float motorGearRatio;
        float motorPosition;
        float jointPosition;
    public:
        Motor(){};
        Motor(uint8_t num, TIM_HandleTypeDef, uint32_t){
            this->motorNumber = num;
        }
        
        void setPWMTimer(TIM_HandleTypeDef);
        void setPWMTimerChannel(uint32_t);
        
        void setDefaultDirection(uint8_t);
        void setDirectionGpioPort(GPIO_TypeDef*);
        void setDirectionGpioPin(uint16_t);
        
        void setEncoderTimer(TIM_HandleTypeDef);
        
        void setNumber(uint8_t);
        void setPWM(int16_t);
        void setDirection(uint8_t);
        
        
        uint8_t getNumber(void);
        int16_t getEncoderCount(void);
        
        float getMotorRPM(void);
        float getMotorAngVel(void);
        float getMotorPosition(void);
        
        float getJointRPM(void);
        float getJointAngVel(void);
        float getJointPosition(void);
        
        void setGearRatio(float);
};

#endif /* MOTOR_H */