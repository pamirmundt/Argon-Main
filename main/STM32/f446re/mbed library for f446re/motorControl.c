/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* External fucnitons --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private Variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

/* Functions ---------------------------------------------------------------- */

void setMotorPWM(uint32_t motorNumber, uint16_t pwm){ 
    //TIM_CHANNEL_1 - Motor 1 PWM - PB8
    //TIM_CHANNEL_2 - Motor 2 PWM - PB9
    //TIM_CHANNEL_3 - Motor 3 PWM - PB10
    //TIM_CHANNEL_4 - Motor 4 PWM - PB2
    uint32_t channel;
    if(motorNumber == 1)
        channel = TIM_CHANNEL_1;
    else if(motorNumber == 2)
        channel = TIM_CHANNEL_2;
    else if(motorNumber == 3)
        channel = TIM_CHANNEL_3;
    else if(motorNumber == 4)
        channel = TIM_CHANNEL_4;
    else
        return;
    
    if(pwm <= 4096)
        __HAL_TIM_SET_COMPARE(&htim2, channel, pwm);
    else
        __HAL_TIM_SET_COMPARE(&htim2, channel, 4096);
}

/* Set Motor Directions
    Motor 1 Direction Pin - PB12
    Motor 2 Direction Pin - PB13
    Motor 3 Direction Pin - PB14
    Motor 4 Direction Pin - PB15
*/
void setMotorDirection(uint8_t motorNumber, uint8_t dir){
    uint16_t dir_pin;
    GPIO_PinState state = GPIO_PIN_RESET;
    
    if(motorNumber == 1){
        dir_pin = GPIO_PIN_12;
        if(dir)
            state = GPIO_PIN_SET;
    }
    
    else if(motorNumber == 2){
        dir_pin = GPIO_PIN_13;
        if(!dir)
            state = GPIO_PIN_SET;
    }
    
    else if(motorNumber == 3){
        dir_pin = GPIO_PIN_14;
        if(dir)
            state = GPIO_PIN_SET;
    }
        
        
    else if(motorNumber == 4){
        dir_pin = GPIO_PIN_15;
        if(!dir)
            state = GPIO_PIN_SET;    
    }
    else
        return;

    HAL_GPIO_WritePin(GPIOB, dir_pin, state);
}