/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* External fucnitons --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern void MX_GPIO_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TIM15_Init(void); 

/* Private Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

/* Functions ---------------------------------------------------------------- */
void baseInit(void){
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();    
}

void baseEncoderStart(){
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
       
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);  
}

void baseRpmStart(){
    HAL_TIM_Base_Start_IT(&htim9);  
}

void basePwmStart(){
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); 
}