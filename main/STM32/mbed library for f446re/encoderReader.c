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
//Get Encoder Count
int16_t getEncoderCount(uint8_t motorNumber){
    //Front Left Wheel
    if(motorNumber == 1)
        return encoder_count_CH1;
    //Front Right Wheel
    else if(motorNumber == 2)
        return encoder_count_CH2;
    //Rear Left Wheel
    else if(motorNumber == 3)
        return encoder_count_CH3;
    //Rear Right Wheel
    else
        return encoder_count_CH4;
};

//Get Clock Difference between encoder ticks
//Get Delta Clock Encoder
uint16_t getDeltaClkEncoder(uint8_t motorNumber, uint8_t channelNumber){
    //Motor 1 - TIM1 CH1 / TIM8 CH1
    if(motorNumber == 1 && channelNumber == 1)
        return delta_clk_TIM1_CH1;
    else if(motorNumber == 1 && channelNumber == 2)
        return delta_clk_TIM8_CH1;
        
    //Motor 2 - TIM1 CH2 / TIM8 CH2
    else if(motorNumber == 2 && channelNumber == 1)
        return delta_clk_TIM1_CH2;
    else if(motorNumber == 2 && channelNumber == 2)
        return delta_clk_TIM8_CH2;
    
    //Motor 3 - TIM1 CH3 / TIM8 CH3
    else if(motorNumber == 3 && channelNumber == 1)
        return delta_clk_TIM1_CH3;
    else if(motorNumber == 3 && channelNumber == 2)
        return delta_clk_TIM8_CH3;
    
    //Motor 4 - TIM1 CH4 / TIM8 CH4
    else if(motorNumber == 4 && channelNumber == 1)
        return delta_clk_TIM1_CH4;
    else if(motorNumber == 4 && channelNumber == 2)
        return delta_clk_TIM8_CH4;
    
    //Else (ERROR)
    else
        return 0;
}

//Clear Clock Difference between encoder ticks
//Clear Delta Clock Encoder
void clearDeltaClkEncoder(uint8_t motorNumber, uint8_t channelNumber){
    //Motor 1 - TIM1 CH1 / TIM8 CH1
    if(motorNumber == 1 && channelNumber == 1)
        delta_clk_TIM1_CH1 = 0;
    else if(motorNumber == 1 && channelNumber == 2)
        delta_clk_TIM8_CH1 = 0;
        
    //Motor 2 - TIM1 CH2 / TIM8 CH2
    else if(motorNumber == 2 && channelNumber == 1)
        delta_clk_TIM1_CH2 = 0;
    else if(motorNumber == 2 && channelNumber == 2)
        delta_clk_TIM8_CH2 = 0;
    
    //Motor 3 - TIM1 CH3 / TIM8 CH3
    else if(motorNumber == 3 && channelNumber == 1)
        delta_clk_TIM1_CH3 = 0;
    else if(motorNumber == 3 && channelNumber == 2)
        delta_clk_TIM8_CH3 = 0;
    
    //Motor 4 - TIM1 CH4 / TIM8 CH4
    else if(motorNumber == 4 && channelNumber == 1)
        delta_clk_TIM1_CH4 = 0;
    else if(motorNumber == 4 && channelNumber == 2)
        delta_clk_TIM8_CH4 = 0;
    
    //Else (ERROR)
    else
        return;
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
            encoder_count_CH1--;
            
            uint16_t input_capture_TIM1_CH1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM2 channel 1 capture value
            delta_clk_TIM1_CH1 = input_capture_TIM1_CH1 - prev_capture_TIM1_CH1;
            prev_capture_TIM1_CH1 = input_capture_TIM1_CH1;
            
            delta_clk_TIM8_CH1 = 0;
        }
        
        //Timer 1 - Channel 2
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            encoder_count_CH2++;
            
            uint16_t input_capture_TIM1_CH2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); //read TIM2 channel 2 capture value
            delta_clk_TIM1_CH2 = input_capture_TIM1_CH2 - prev_capture_TIM1_CH2;
            prev_capture_TIM1_CH2 = input_capture_TIM1_CH2;
            
            delta_clk_TIM8_CH2 = 0;
        }
        
        //Timer 1 - Channel 3
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            encoder_count_CH3--;
            
            uint16_t input_capture_TIM1_CH3 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3); //read TIM2 channel 3 capture value
            delta_clk_TIM1_CH3 = input_capture_TIM1_CH3 - prev_capture_TIM1_CH3;
            prev_capture_TIM1_CH3 = input_capture_TIM1_CH3;
            
            delta_clk_TIM8_CH3 = 0;
        }
        
        //Timer 1 - Channel 4
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            encoder_count_CH4++;
            
            uint16_t input_capture_TIM1_CH4 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4); //read TIM2 channel 4 capture value
            delta_clk_TIM1_CH4 = input_capture_TIM1_CH4 - prev_capture_TIM1_CH4;
            prev_capture_TIM1_CH4 = input_capture_TIM1_CH4;
            
            delta_clk_TIM8_CH4 = 0;
        }
    }
    
    else if(htim->Instance == TIM8){
        //Timer 8 - Channel 1
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            encoder_count_CH1++;
            
            uint16_t input_capture_TIM8_CH1 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1); //read TIM3 channel 1 capture value
            delta_clk_TIM8_CH1 = input_capture_TIM8_CH1 - prev_capture_TIM8_CH1;
            prev_capture_TIM8_CH1 = input_capture_TIM8_CH1; 
            
            delta_clk_TIM1_CH1 = 0;
        }
        
        //Timer 8 - Channel 2
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            encoder_count_CH2--;
            
            uint16_t input_capture_TIM8_CH2 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2); //read TIM3 channel 2 capture value
            delta_clk_TIM8_CH2 = input_capture_TIM8_CH2 - prev_capture_TIM8_CH2;
            prev_capture_TIM8_CH2 = input_capture_TIM8_CH2;
            
            delta_clk_TIM1_CH2 = 0;
        }
        
        //Timer 8 - Channel 3
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            encoder_count_CH3++;
            
            uint16_t input_capture_TIM8_CH3 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3); //read TIM3 channel 3 capture value
            delta_clk_TIM8_CH3 = input_capture_TIM8_CH3 - prev_capture_TIM8_CH3;
            prev_capture_TIM8_CH3 = input_capture_TIM8_CH3;
            
            delta_clk_TIM1_CH3 = 0;
        }
        
        //Timer 8 - Channel 4
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            encoder_count_CH4--;
            
            uint16_t input_capture_TIM8_CH4 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4); //read TIM3 channel 3 capture value
            delta_clk_TIM8_CH4 = input_capture_TIM8_CH4 - prev_capture_TIM8_CH4;
            prev_capture_TIM8_CH4 = input_capture_TIM8_CH4;
            
            delta_clk_TIM1_CH4 = 0;
        }
    }
}