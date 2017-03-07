/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "encoderReader.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/* Private Variables ---------------------------------------------------------*/
volatile int16_t motor1_encoder = 0, motor2_encoder = 0, motor3_encoder = 0, motor4_encoder = 0;

volatile uint16_t prev_capture_TIM1_CH1 = 0, prev_capture_TIM1_CH2 = 0, prev_capture_TIM1_CH3 = 0, prev_capture_TIM1_CH4 = 0;
volatile uint16_t prev_capture_TIM3_CH1 = 0, prev_capture_TIM3_CH2 = 0, prev_capture_TIM3_CH3 = 0, prev_capture_TIM3_CH4 = 0;

volatile uint16_t delta_clk_TIM1_CH1 = 0, delta_clk_TIM1_CH2 = 0, delta_clk_TIM1_CH3 = 0, delta_clk_TIM1_CH4 = 0;
volatile uint16_t delta_clk_TIM3_CH1 = 0, delta_clk_TIM3_CH2 = 0, delta_clk_TIM3_CH3 = 0, delta_clk_TIM3_CH4 = 0;

/* Functions ---------------------------------------------------------------- */
int16_t get_encoder1_count(void){
    return motor1_encoder;
}

int16_t get_encoder2_count(void){
    return motor2_encoder;
}

int16_t get_encoder3_count(void){
    return motor3_encoder;
}

int16_t get_encoder4_count(void){
    return motor4_encoder;
}

//Get TIM1 Channels
uint16_t get_delta_clk_TIM1_CH1(void){
    return delta_clk_TIM1_CH1;    
}

uint16_t get_delta_clk_TIM1_CH2(void){
    return delta_clk_TIM1_CH2;    
}

uint16_t get_delta_clk_TIM1_CH3(void){
    return delta_clk_TIM1_CH3;    
}

uint16_t get_delta_clk_TIM1_CH4(void){
    return delta_clk_TIM1_CH4;    
}

//Get TIM3 Channels
uint16_t get_delta_clk_TIM3_CH1(void){
    return delta_clk_TIM3_CH1;    
}

uint16_t get_delta_clk_TIM3_CH2(void){
    return delta_clk_TIM3_CH2;    
}

uint16_t get_delta_clk_TIM3_CH3(void){
    return delta_clk_TIM3_CH3;    
}

uint16_t get_delta_clk_TIM3_CH4(void){
    return delta_clk_TIM3_CH4;    
}

//Clear TIM1 Channels
void clear_delta_clk_TIM1_CH1(void){
    delta_clk_TIM1_CH1 = 0;
}

void clear_delta_clk_TIM1_CH2(void){
    delta_clk_TIM1_CH2 = 0;
}

void clear_delta_clk_TIM1_CH3(void){
    delta_clk_TIM1_CH3 = 0;
}

void clear_delta_clk_TIM1_CH4(void){
    delta_clk_TIM1_CH4 = 0;
}

//Clear TIM3 Channels
void clear_delta_clk_TIM3_CH1(void){
    delta_clk_TIM3_CH1 = 0;
}

void clear_delta_clk_TIM3_CH2(void){
    delta_clk_TIM3_CH2 = 0;
}

void clear_delta_clk_TIM3_CH3(void){
    delta_clk_TIM3_CH3 = 0;
}

void clear_delta_clk_TIM3_CH4(void){
    delta_clk_TIM3_CH4 = 0;
}

/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim: TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        //Timer 1 - Channel 1
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            motor1_encoder++;
            
            uint16_t input_capture_TIM1_CH1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH1 = input_capture_TIM1_CH1 - prev_capture_TIM1_CH1;
            prev_capture_TIM1_CH1 = input_capture_TIM1_CH1;
        }
        //Timer 1 - Channel 2
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            motor2_encoder++;
            
            uint16_t input_capture_TIM1_CH2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH2 = input_capture_TIM1_CH2 - prev_capture_TIM1_CH2;
            prev_capture_TIM1_CH2 = input_capture_TIM1_CH2;
        }
        //Timer 1 - Channel 3
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            motor3_encoder++;
            
            uint16_t input_capture_TIM1_CH3 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH3 = input_capture_TIM1_CH3 - prev_capture_TIM1_CH3;
            prev_capture_TIM1_CH3 = input_capture_TIM1_CH3;
        }
        //Timer 1 - Channel 4
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            motor4_encoder++;
            
            uint16_t input_capture_TIM1_CH4 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH4 = input_capture_TIM1_CH4 - prev_capture_TIM1_CH4;
            prev_capture_TIM1_CH4 = input_capture_TIM1_CH4;
        }
    }
    if(htim->Instance == TIM3){
        //Timer 3 - Channel 1
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            motor1_encoder--;
            
            uint16_t input_capture_TIM3_CH1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1); //read TIM3 channel 1 capture value
            delta_clk_TIM3_CH1 = input_capture_TIM3_CH1 - prev_capture_TIM3_CH1;
            prev_capture_TIM3_CH1 = input_capture_TIM3_CH1;
        }
        //Timer 3 - Channel 2
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            motor2_encoder--;
            
            uint16_t input_capture_TIM3_CH2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2); //read TIM3 channel 2 capture value
            delta_clk_TIM3_CH2 = input_capture_TIM3_CH2 - prev_capture_TIM3_CH2;
            prev_capture_TIM3_CH2 = input_capture_TIM3_CH2;
        }
        //Timer 3 - Channel 3
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            motor3_encoder--;
            
            uint16_t input_capture_TIM3_CH3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3); //read TIM3 channel 3 capture value
            delta_clk_TIM3_CH3 = input_capture_TIM3_CH3 - prev_capture_TIM3_CH3;
            prev_capture_TIM3_CH3 = input_capture_TIM3_CH3;
        }
        //Timer 3 - Channel 4
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            motor4_encoder--;
            
            uint16_t input_capture_TIM3_CH4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4); //read TIM3 channel 3 capture value
            delta_clk_TIM3_CH4 = input_capture_TIM3_CH4 - prev_capture_TIM3_CH4;
            prev_capture_TIM3_CH4 = input_capture_TIM3_CH4;
        }
    }
 }