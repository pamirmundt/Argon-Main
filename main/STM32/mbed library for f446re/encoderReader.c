/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "encoderReader.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* Private Variables ---------------------------------------------------------*/
volatile int16_t encoder_count_CH1 = 0, encoder_count_CH2 = 0, encoder_count_CH3 = 0, encoder_count_CH4 = 0;

volatile uint16_t prev_capture_TIM1_CH1 = 0, prev_capture_TIM1_CH2 = 0, prev_capture_TIM1_CH3 = 0, prev_capture_TIM1_CH4 = 0;
volatile uint16_t prev_capture_TIM8_CH1 = 0, prev_capture_TIM8_CH2 = 0, prev_capture_TIM8_CH3 = 0, prev_capture_TIM8_CH4 = 0;

volatile uint16_t delta_clk_TIM1_CH1 = 0, delta_clk_TIM1_CH2 = 0, delta_clk_TIM1_CH3 = 0, delta_clk_TIM1_CH4 = 0;
volatile uint16_t delta_clk_TIM8_CH1 = 0, delta_clk_TIM8_CH2 = 0, delta_clk_TIM8_CH3 = 0, delta_clk_TIM8_CH4 = 0;

/* Functions ---------------------------------------------------------------- */
int16_t get_encoder1_count(void){
    return encoder_count_CH1;
}

int16_t get_encoder2_count(void){
    return encoder_count_CH2;
}

int16_t get_encoder3_count(void){
    return encoder_count_CH3;
}

int16_t get_encoder4_count(void){
    return encoder_count_CH4;
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

//Get TIM8 Channels
uint16_t get_delta_clk_TIM8_CH1(void){
    return delta_clk_TIM8_CH1;    
}

uint16_t get_delta_clk_TIM8_CH2(void){
    return delta_clk_TIM8_CH2;    
}

uint16_t get_delta_clk_TIM8_CH3(void){
    return delta_clk_TIM8_CH3;    
}

uint16_t get_delta_clk_TIM8_CH4(void){
    return delta_clk_TIM8_CH4;    
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

//Clear TIM8 Channels
void clear_delta_clk_TIM8_CH1(void){
    delta_clk_TIM8_CH1 = 0;
}

void clear_delta_clk_TIM8_CH2(void){
    delta_clk_TIM8_CH2 = 0;
}

void clear_delta_clk_TIM8_CH3(void){
    delta_clk_TIM8_CH3 = 0;
}

void clear_delta_clk_TIM8_CH4(void){
    delta_clk_TIM8_CH4 = 0;
}

/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim: TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        //Timer 1 - Channel 1
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            encoder_count_CH1++;
            
            uint16_t input_capture_TIM1_CH1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH1 = input_capture_TIM1_CH1 - prev_capture_TIM1_CH1;
            prev_capture_TIM1_CH1 = input_capture_TIM1_CH1;
        }
        
        //Timer 1 - Channel 2
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            encoder_count_CH2++;
            
            uint16_t input_capture_TIM1_CH2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); //read TIM2 channel 2 capture value
            delta_clk_TIM1_CH2 = input_capture_TIM1_CH2 - prev_capture_TIM1_CH2;
            prev_capture_TIM1_CH2 = input_capture_TIM1_CH2;
        }
        
        //Timer 1 - Channel 3
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            encoder_count_CH3++;
            
            uint16_t input_capture_TIM1_CH3 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3); //read TIM2 channel 3 capture value
            delta_clk_TIM1_CH3 = input_capture_TIM1_CH3 - prev_capture_TIM1_CH3;
            prev_capture_TIM1_CH3 = input_capture_TIM1_CH3;
        }
        
        //Timer 1 - Channel 4
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            encoder_count_CH4++;
            
            uint16_t input_capture_TIM1_CH4 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4); //read TIM2 channel 4 capture value
            delta_clk_TIM1_CH4 = input_capture_TIM1_CH4 - prev_capture_TIM1_CH4;
            prev_capture_TIM1_CH4 = input_capture_TIM1_CH4;
        }
    }
    
    else if(htim->Instance == TIM8){
        //Timer 8 - Channel 1
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            encoder_count_CH1--;
            
            uint16_t input_capture_TIM8_CH1 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1); //read TIM3 channel 1 capture value
            delta_clk_TIM8_CH1 = input_capture_TIM8_CH1 - prev_capture_TIM8_CH1;
            prev_capture_TIM8_CH1 = input_capture_TIM8_CH1; 
        }
        
        //Timer 8 - Channel 2
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            encoder_count_CH2--;
            
            uint16_t input_capture_TIM8_CH2 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2); //read TIM3 channel 2 capture value
            delta_clk_TIM8_CH2 = input_capture_TIM8_CH2 - prev_capture_TIM8_CH2;
            prev_capture_TIM8_CH2 = input_capture_TIM8_CH2;
        }
        
        //Timer 8 - Channel 3
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            encoder_count_CH3--;
            
            uint16_t input_capture_TIM8_CH3 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3); //read TIM3 channel 3 capture value
            delta_clk_TIM8_CH3 = input_capture_TIM8_CH3 - prev_capture_TIM8_CH3;
            prev_capture_TIM8_CH3 = input_capture_TIM8_CH3;
        }
        
        //Timer 8 - Channel 4
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            encoder_count_CH4--;
            
            uint16_t input_capture_TIM8_CH4 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4); //read TIM3 channel 3 capture value
            delta_clk_TIM8_CH4 = input_capture_TIM8_CH4 - prev_capture_TIM8_CH4;
            prev_capture_TIM8_CH4 = input_capture_TIM8_CH4;
        }
    }
}