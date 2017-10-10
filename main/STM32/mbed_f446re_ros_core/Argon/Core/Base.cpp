/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

#include "Base.h"

/* External fucnitons --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern void MX_GPIO_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TIM4_Init(void);
extern void MX_TIM8_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_TIM6_Init(void);

/* Private Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* Prototype Functions -------------------------------------------------------*/
void baseInit(void);
void baseEncoderStart(void);
void baseRpmStart(void);
void basePwmStart(void);

/* Functions ---------------------------------------------------------------- */
void baseInit(void){
    //initialize GPIO and Timers
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();
    MX_TIM2_Init();
    MX_TIM6_Init();
    
    //Start PWM Timers
    basePwmStart();
    //Start Encoder Timers
    baseEncoderStart();
    //Start RPM Calculation Timer
    baseRpmStart();
}

void baseEncoderStart(void){
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);  
}

void baseRpmStart(void){
    HAL_TIM_Base_Start_IT(&htim6);  
}

void basePwmStart(void){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
}