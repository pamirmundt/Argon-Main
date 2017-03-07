/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* External fucnitons --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern void MX_GPIO_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TIM15_Init(void); 

/* Private Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* Functions ---------------------------------------------------------------- */
void base_init(void){
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM15_Init();    
}

void base_start(){
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
       
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
    
    HAL_TIM_Base_Start_IT(&htim15);    
}