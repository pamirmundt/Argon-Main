#ifndef BASE_H
#define BASE_H


/* Includes ------------------------------------------------------------------*/
#include "Inits.h"
#include "params.h"
#include "Motor.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Prototype functions ------------------------------------------------------ */
void baseInit(void);
//void baseEncoderStart(void);
//void baseRpmStart(void);
//void basePwmStart(void);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

class Base{
    public:
        Motor frontLeft;
        Motor frontRight;
        Motor rearLeft;
        Motor rearRight;
        Base(){
            baseInit();
            this->frontLeft.setNumber(1);
            this->frontRight.setNumber(2);
            this->rearLeft.setNumber(3);
            this->rearRight.setNumber(4);
            
            this->frontLeft.setGearRatio(gearRatio);
            this->frontLeft.setPWMTimer(htim2);
            this->frontLeft.setPWMTimerChannel(TIM_CHANNEL_1);
            this->frontLeft.setDefaultDirection(0);
            this->frontLeft.setDirectionGpioPort(motorDirection1_GPIO_Port);
            this->frontLeft.setDirectionGpioPin(motorDirection1_Pin);
            this->frontLeft.setEncoderTimer(htim1);
            
            this->frontRight.setGearRatio(gearRatio);
            this->frontRight.setPWMTimer(htim2);
            this->frontRight.setPWMTimerChannel(TIM_CHANNEL_2);
            this->frontRight.setDefaultDirection(1);
            this->frontRight.setDirectionGpioPort(motorDirection2_GPIO_Port);
            this->frontRight.setDirectionGpioPin(motorDirection2_Pin);
            this->frontRight.setEncoderTimer(htim3);
            
            this->rearLeft.setGearRatio(gearRatio);
            this->rearLeft.setPWMTimer(htim2);
            this->rearLeft.setPWMTimerChannel(TIM_CHANNEL_3);
            this->rearLeft.setDefaultDirection(0);
            this->rearLeft.setDirectionGpioPort(motorDirection3_GPIO_Port);
            this->rearLeft.setDirectionGpioPin(motorDirection3_Pin);
            this->rearLeft.setEncoderTimer(htim4);

            this->rearRight.setGearRatio(gearRatio);
            this->rearRight.setPWMTimer(htim2);
            this->rearRight.setPWMTimerChannel(TIM_CHANNEL_4);
            this->rearRight.setDefaultDirection(1);
            this->rearRight.setDirectionGpioPort(motorDirection4_GPIO_Port);
            this->rearRight.setDirectionGpioPin(motorDirection4_Pin);
            this->rearRight.setEncoderTimer(htim8);
        }
};

#endif /* Base_H */
