#include <stdlib.h>
#include "Motor.h"
#include "Inits.h"
#include "params.h"


/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

/* Defines -------------------------------------------------------------------*/
#define M_PI        3.14159265358979323846f
#define RPM2RAD     ((2.0f*M_PI)/60.0f)
#define TICK2RAD    ((360.0f/encoder_resolution/encoder_mode)*(M_PI/180.0f))

/*******************************************************************************
    RPM Resolution per encoder tick
        (Encoder tick/sec) to RPM
    Example:  1  encoder tick per second = 187,5 RPM
              2  encoder tick per second = 375,0 RPM

                        60sec * (Frequency of RPM calculation timer)
    RPM_res_per_tick = ----------------------------------------------
                        Encoder Resolution * Encoder Reading Mode(x4)
*******************************************************************************/
#define RPM_res_per_tick (60.0f * Timer6Frequency) / (encoder_resolution*encoder_mode)

/* Private Variables ---------------------------------------------------------*/
float motor1RPM = 0.0f, motor2RPM = 0.0f, motor3RPM = 0.0f, motor4RPM = 0.0f;
float filteredMotorRPM1 = 0.0f, filteredMotorRPM2 = 0.0f, filteredMotorRPM3 = 0.0f, filteredMotorRPM4 = 0.0f;
int16_t prevMotor1EncoderCount = 0, prevMotor2EncoderCount = 0, prevMotor3EncoderCount = 0, prevMotor4EncoderCount = 0;

//Radians
volatile float wheelPositionFL = 0.0f, wheelPositionFR = 0.0f, wheelPositionRL = 0.0f, wheelPositionRR = 0.0f;

//
void Motor::setPWMTimer(TIM_HandleTypeDef _htimerPWM){
    htimerPWM = _htimerPWM;
}

//
void Motor::setPWMTimerChannel(uint32_t _htimerChannelPWM){
    htimerChannelPWM = _htimerChannelPWM;
}

//
void Motor::setDefaultDirection(uint8_t _defaultDirection){
    defaultDirection = _defaultDirection;
}

//
void Motor::setDirectionGpioPort(GPIO_TypeDef * _directionGpioPort){
    directionGpioPort = _directionGpioPort;
}

//
void Motor::setDirectionGpioPin(uint16_t _directionGpioPin){
    directionGpioPin = _directionGpioPin;
}

//
void Motor::setEncoderTimer(TIM_HandleTypeDef _htimerEncoder){
    htimerEncoder = _htimerEncoder;
}

//
void Motor::setNumber(uint8_t _motorNumber){
    motorNumber = _motorNumber;
}

//
uint8_t Motor::getNumber(void){
    return (this->motorNumber);
}

//
void Motor::setPWM(int16_t PWM){
    //Direction
    if(PWM >= 0)
        setDirection(backward);
    else
        setDirection(forward);
    
    //Write PWM
    __HAL_TIM_SET_COMPARE(&this->htimerPWM, this->htimerChannelPWM, abs(PWM));
}

//
void Motor::setDirection(uint8_t dir){
    GPIO_PinState state = GPIO_PIN_RESET;
    if(this->defaultDirection)
        if(dir)
            state = GPIO_PIN_SET;
        else
            state = GPIO_PIN_RESET;
    else
        if(dir)
            state = GPIO_PIN_RESET;
        else
            state = GPIO_PIN_SET;
    
    HAL_GPIO_WritePin(this->directionGpioPort, this->directionGpioPin, state);
}

//
int16_t Motor::getEncoderCount(void){    
    if(this->defaultDirection)
        return (this->htimerEncoder.Instance->CNT);
    else
        return (65536 - this->htimerEncoder.Instance->CNT);
}


float Motor::getMotorRPM(void){
    float RPM = 0.0f;
    switch(this->motorNumber){
    case 1:
        if(RPM_MAF)
            RPM = filteredMotorRPM1;
        else
            RPM = motor1RPM;
        break;
    case 2:
        if(RPM_MAF)
            RPM = filteredMotorRPM2;
        else
            RPM = motor2RPM;
        break;
    case 3:
        if(RPM_MAF)
            RPM = filteredMotorRPM3;
        else
            RPM = motor3RPM;
        break;
    case 4:
        if(RPM_MAF)
            RPM = filteredMotorRPM4;
        else
            RPM = motor4RPM;
        break;
    default:
        return -1;
    }
    return RPM;
}

//
float Motor::getMotorAngVel(void){
    return (RPM2RAD*(this->getMotorRPM()));
}

//
float Motor::getMotorPosition(void){
    switch(this->motorNumber){
    case 1:
        return wheelPositionFL;
    case 2:
        return wheelPositionFR;
    case 3:
        return wheelPositionRL;
    case 4:
        return wheelPositionRR;
    default:
        return -1;
    }
}

//
float Motor::getJointRPM(void){
    return (this->getMotorRPM() / this->motorGearRatio);
}

//
float Motor::getJointAngVel(void){
    return (RPM2RAD*this->getJointRPM());
}

float Motor::getJointPosition(void){
    return (this->getMotorPosition() / this->motorGearRatio);
}

//
void Motor::setGearRatio(float _motorGearRatio){
    motorGearRatio = _motorGearRatio;
}

//Calculate RPM
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6){
        //Motor 1
        int16_t motor1EncoderCount = 65536 - htim1.Instance->CNT;
        int16_t deltaMotor1EncoderCount = motor1EncoderCount - prevMotor1EncoderCount;
        motor1RPM = RPM_res_per_tick * ((float)((int16_t)deltaMotor1EncoderCount));
        if (RPM_MAF)
            filteredMotorRPM1 = (RPM_MAF_Alpha*motor1RPM)+(1.0f - RPM_MAF_Alpha)*filteredMotorRPM1;
        prevMotor1EncoderCount = motor1EncoderCount;
        
        //Motor 2
        int16_t motor2EncoderCount = htim3.Instance->CNT;
        int16_t deltaMotor2EncoderCount = motor2EncoderCount - prevMotor2EncoderCount;
        motor2RPM = RPM_res_per_tick * ((float)((int16_t)(deltaMotor2EncoderCount)));
        if (RPM_MAF)
            filteredMotorRPM2 = (RPM_MAF_Alpha*motor2RPM)+(1.0f - RPM_MAF_Alpha)*filteredMotorRPM2;
        prevMotor2EncoderCount = motor2EncoderCount;
        
        //Motor 3
        int16_t motor3EncoderCount = 65536 - htim4.Instance->CNT;
        int16_t deltaMotor3EncoderCount = motor3EncoderCount - prevMotor3EncoderCount;
        motor3RPM = RPM_res_per_tick * ((float)((int16_t)(deltaMotor3EncoderCount)));
        if (RPM_MAF)
            filteredMotorRPM3 = (RPM_MAF_Alpha*motor3RPM)+(1.0f - RPM_MAF_Alpha)*filteredMotorRPM3;
        prevMotor3EncoderCount = motor3EncoderCount;
        
        //Motor 4
        int16_t motor4EncoderCount = htim8.Instance->CNT;
        int16_t deltaMotor4EncoderCount = motor4EncoderCount - prevMotor4EncoderCount;
        motor4RPM = RPM_res_per_tick * ((float)((int16_t)(deltaMotor4EncoderCount)));
        if (RPM_MAF)
            filteredMotorRPM4 = (RPM_MAF_Alpha*motor4RPM)+(1.0f - RPM_MAF_Alpha)*filteredMotorRPM4;
        prevMotor4EncoderCount = motor4EncoderCount;
        
        //Wheel Positions
        wheelPositionFL += (TICK2RAD * ((float)deltaMotor1EncoderCount));
        wheelPositionFR += (TICK2RAD * ((float)deltaMotor2EncoderCount));
        wheelPositionRL += (TICK2RAD * ((float)deltaMotor3EncoderCount));
        wheelPositionRR += (TICK2RAD * ((float)deltaMotor4EncoderCount));
    }
}
