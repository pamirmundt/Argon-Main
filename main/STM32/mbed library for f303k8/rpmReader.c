/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "rpmReader.h"

/* External variables --------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint16_t prev_encoder_count_CH1 = 0, prev_encoder_count_CH2 = 0, prev_encoder_count_CH3 = 0, prev_encoder_count_CH4 = 0;
volatile uint8_t timeout_CH1 = 0, timeout_CH2 = 0, timeout_CH3 = 0, timeout_CH4 = 0;

volatile float RPM_CH1 = 0.0f, RPM_CH2 = 0.0f, RPM_CH3 = 0.0f, RPM_CH4 = 0.0f;

/* Functions variables -------------------------------------------------------*/

float get_motor1_RPM(void){
    return RPM_CH1;    
}

float get_motor2_RPM(void){
    return RPM_CH2;    
}

float get_motor3_RPM(void){
    return RPM_CH3;    
}

float get_motor4_RPM(void){
    return RPM_CH4;    
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM15)
    {
        //----------------------------------------------------------------------
        //  Channel 1 RPM
        //----------------------------------------------------------------------
        //Timeout for CH1
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH1 = get_delta_clk_TIM1_CH1();
        uint16_t delta_clk_TIM3_CH1 = get_delta_clk_TIM3_CH1();
        int16_t encoder_count_CH1 = get_encoder1_count();
        int16_t delta_encoder_CH1 = encoder_count_CH1 - prev_encoder_count_CH1;
        if(delta_encoder_CH1 == 0){
            timeout_CH1++;
            if(timeout_CH1 == 200){
                clear_delta_clk_TIM1_CH1();
                clear_delta_clk_TIM3_CH1();
                timeout_CH1 = 0;
            }
        }
        else
            timeout_CH1 = 0;
        prev_encoder_count_CH1 = encoder_count_CH1;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 2 - Channel 1
        if(delta_clk_TIM1_CH1 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH1 = 60000.0f/(float)delta_clk_TIM1_CH1;
        }
        //Timer 3 - Channel 1
        else if(delta_clk_TIM3_CH1 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH1 = -60000.0f/(float)delta_clk_TIM3_CH1;
        }
        else if((delta_clk_TIM1_CH1 < 320) || (delta_clk_TIM3_CH1 < 320))
            RPM_CH1 = 187.5f*((float)delta_encoder_CH1);
            
        
        //----------------------------------------------------------------------
        //  Channel 2 RPM
        //----------------------------------------------------------------------
        //Timeout for CH2
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH2 = get_delta_clk_TIM1_CH2();
        uint16_t delta_clk_TIM3_CH2 = get_delta_clk_TIM3_CH2();
        int16_t encoder_count_CH2 = get_encoder2_count();
        int16_t delta_encoder_CH2 = encoder_count_CH2 - prev_encoder_count_CH2;
        if(delta_encoder_CH2 == 0){
            timeout_CH2++;
            if(timeout_CH2 == 200){
                clear_delta_clk_TIM1_CH2();
                clear_delta_clk_TIM3_CH2();
                timeout_CH2 = 0;
            }
        }
        else
            timeout_CH2 = 0;
        prev_encoder_count_CH2 = encoder_count_CH2;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 1 - Channel 2
        if(delta_clk_TIM1_CH2 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH2 = 60000.0f/(float)delta_clk_TIM1_CH2;
        }
        //Timer 3 - Channes 3
        else if(delta_clk_TIM3_CH2 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH2 = -60000.0f/(float)delta_clk_TIM3_CH2;
        }
        else if((delta_clk_TIM1_CH2 < 320) || (delta_clk_TIM3_CH2 < 320))
            RPM_CH2 = 187.5f*((float)delta_encoder_CH2);
            
        //----------------------------------------------------------------------
        //  Channel 3 RPM
        //----------------------------------------------------------------------
        //Timeout for CH3
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH3 = get_delta_clk_TIM1_CH3();
        uint16_t delta_clk_TIM3_CH3 = get_delta_clk_TIM3_CH3();
        int16_t encoder_count_CH3 = get_encoder3_count();
        int16_t delta_encoder_CH3 = encoder_count_CH3 - prev_encoder_count_CH3;
        if(delta_encoder_CH3 == 0){
            timeout_CH3++;
            if(timeout_CH3 == 200){
                clear_delta_clk_TIM1_CH3();
                clear_delta_clk_TIM3_CH3();
                timeout_CH3 = 0;
            }
        }
        else
            timeout_CH3 = 0;
        prev_encoder_count_CH3 = encoder_count_CH3;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 1 - Channel 3
        if(delta_clk_TIM1_CH3 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH3 = 60000.0f/(float)delta_clk_TIM1_CH3;
        }
        //Timer 3 - Channes 3
        else if(delta_clk_TIM3_CH3 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH3 = -60000.0f/(float)delta_clk_TIM3_CH3;
        }
        else if((delta_clk_TIM1_CH3 < 320) || (delta_clk_TIM3_CH3 < 320))
            RPM_CH3 = 187.5f*((float)delta_encoder_CH3);
            
        //----------------------------------------------------------------------
        //  Channel 4 RPM
        //----------------------------------------------------------------------
        //Timeout for CH4
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH4 = get_delta_clk_TIM1_CH4();
        uint16_t delta_clk_TIM3_CH4 = get_delta_clk_TIM3_CH4();
        int16_t encoder_count_CH4 = get_encoder4_count();
        int16_t delta_encoder_CH4 = encoder_count_CH4 - prev_encoder_count_CH4;
        if(delta_encoder_CH4 == 0){
            timeout_CH4++;
            if(timeout_CH4 == 200){
                clear_delta_clk_TIM1_CH4();
                clear_delta_clk_TIM3_CH4();
                timeout_CH4 = 0;
            }
        }
        else
            timeout_CH4 = 0;
        prev_encoder_count_CH4 = encoder_count_CH4;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 1 - Channel 4
        if(delta_clk_TIM1_CH4 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH4 = 60000.0f/(float)delta_clk_TIM1_CH4;
        }
        //Timer 3 - Channes 4
        else if(delta_clk_TIM3_CH4 >= 320){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH4 = -60000.0f/(float)delta_clk_TIM3_CH4;
        }
        else if((delta_clk_TIM1_CH4 < 320) || (delta_clk_TIM3_CH4 < 320))
            RPM_CH4 = 187.5f*((float)delta_encoder_CH4);
    }
}