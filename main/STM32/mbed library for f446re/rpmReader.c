/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rpmReader.h"
#include "params.h"

/* Defines -------------------------------------------------------------------*/
//PLLCLK = 180Mhz

/*******************************************************************************
                                PLLCLK
    Timer 9 Frequency =  ---------------------------------
                            (Prescaler + 1)*(Period + 1)                            
*******************************************************************************/
#define timer9_freq (PLLCLK_speed/(timer9_prescaler + 1)/ (timer9_period + 1))


/*******************************************************************************
    Input Capture Frequency (Timer 1&8)
    
                        PLLCLK
    IC Frequency = ----------------
                    (Prescaler + 1)
*******************************************************************************/
#define IC_freq (PLLCLK_speed/(IC_prescaler+1))


/*******************************************************************************
    RPM Resolution per encoder tick
        (Encoder tick/sec) to RPM
    Example:  1  encoder tick per second = 187,5 RPM
              2  encoder tick per second = 375,0 RPM

                        60sec * (Frequency of RPM calculation timer)
    RPM_res_per_tick = ----------------------------------------------
                        Encoder Resolution * Encoder Reading Mode(x4)
*******************************************************************************/
#define RPM_res_per_tick ((60.0f*((float)timer9_freq)) / ((float)encoder_resolution*(float)encoder_mode))


/*******************************************************************************
    RPM Resolution per input capture clock tick
        (IC clock tick / Encoder tick) to RPM
    Example:    1   IC tick per encoder tick = 112.500,00 RPM
                100 IC tick per encoder tick = 1.125,00 RPM
                600 IC tick per encoder tick = 187,50 RPM
                
                        60sec * IC Timer Frequency
    RPM_res_pes_IC = -------------------------------------------------
                        Encoder Resolution * Encoder Reading Mode (x4)
*******************************************************************************/
#define RPM_res_per_IC ((60.0f*(float)IC_freq)/((float)encoder_resolution*(float)encoder_mode))


/*******************************************************************************
    Input Capture Threshold
        This threshold determines, how to calculate RPM.
        - Low Input Capture (High RPM): Calculate RPM with encoder ticks.
        - High Input Capture (LOW RPM): Calculate RPM with the time between 2
                                            encoder ticks.
                                            
                    IC Timer Frequency              
    IC_threshol = ---------------------------------------
                    RPM Calculation Frequency (Timer 9)        
*******************************************************************************/
#define IC_threshold (IC_freq/timer9_freq)


/*******************************************************************************
    Input Capture Timeout
        Input capture timer waits for an encoder tick to calculate RPM. If there
            is no tick in XX secs, sets the RPM to zero. XX secs defines the
            minimum RPM that controller can calculate. This value adds a delay
            to calculation of small RPMs.
            - This value is used to calculate time between two different encoder
            ticks.
    Example: 100 IC Ticks (0,50secs) = min 1,875 RPM
             200 IC Ticks (1,00secs) = min 0,9375 RPM
             10  IC Ticks (0,05secs) = min 18,75 RPMS
             
                                
     timeout_compare_value = Timeout Seconds * RPM Calculation Freq. (timer9)
*******************************************************************************/
#define timeout_compare_value (uint16_t)(((float)timeout_msec/1000.0f)*timer9_freq)

/* External variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t prev_encoder_count_CH1 = 0, prev_encoder_count_CH2 = 0, prev_encoder_count_CH3 = 0, prev_encoder_count_CH4 = 0;
volatile uint8_t timeout_CH1 = 0, timeout_CH2 = 0, timeout_CH3 = 0, timeout_CH4 = 0;

volatile float RPM_CH1 = 0.0f, RPM_CH2 = 0.0f, RPM_CH3 = 0.0f, RPM_CH4 = 0.0f;

/* Functions variables -------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
float getMotorRPM(uint8_t motorNumber){
    if(motorNumber == 1)
        return RPM_CH1;
    else if(motorNumber == 2)
        return RPM_CH2;
    else if(motorNumber == 3)
        return RPM_CH3;
    else
        return RPM_CH4;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM9){
        //---------------------------------
        //  Channel 1 RPM
        //---------------------------------
        
        //Timeout for CH1
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH1 = getDeltaClkEncoder(1,1);
        uint16_t delta_clk_TIM8_CH1 = getDeltaClkEncoder(1,2);
        int16_t encoder_count_CH1 = getEncoderCount(1);
        int16_t delta_encoder_CH1 = encoder_count_CH1 - prev_encoder_count_CH1;
        if(delta_encoder_CH1 == 0){
            timeout_CH1++;
            if(timeout_CH1 == timeout_compare_value){
                clearDeltaClkEncoder(1,1);  //Encoder 1 Channel A
                clearDeltaClkEncoder(1,2);  //Encoder 1 Channel B
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
        if(delta_clk_TIM1_CH1 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH1 = RPM_res_per_IC/(float)delta_clk_TIM1_CH1;
        }
        //Timer 3 - Channel 1
        else if(delta_clk_TIM8_CH1 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH1 = -RPM_res_per_IC/(float)delta_clk_TIM8_CH1;
        }
        else if((delta_clk_TIM1_CH1 < IC_threshold) || (delta_clk_TIM8_CH1 < IC_threshold))
            RPM_CH1 = RPM_res_per_tick*((float)delta_encoder_CH1);
        
        //---------------------------------
        //  Channel 2 RPM
        //---------------------------------
        
        //Timeout for CH2
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH2 = getDeltaClkEncoder(2,1);
        uint16_t delta_clk_TIM8_CH2 = getDeltaClkEncoder(2,2);
        int16_t encoder_count_CH2 = getEncoderCount(2);
        int16_t delta_encoder_CH2 = encoder_count_CH2 - prev_encoder_count_CH2;
        if(delta_encoder_CH2 == 0){
            timeout_CH2++;
            if(timeout_CH2 == timeout_compare_value){
                clearDeltaClkEncoder(2,1);  //Encoder 2 Channel A
                clearDeltaClkEncoder(2,2);  //Encoder 2 Channel B
                timeout_CH2 = 0;
            }
        }
        else
            timeout_CH2 = 0;
        prev_encoder_count_CH2 = encoder_count_CH2;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 2 - Channel 2
        if(delta_clk_TIM1_CH2 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH2 = RPM_res_per_IC/(float)delta_clk_TIM1_CH2;
        }
        //Timer 3 - Channes 3
        else if(delta_clk_TIM8_CH2 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH2 = -RPM_res_per_IC/(float)delta_clk_TIM8_CH2;
        }
        else if((delta_clk_TIM1_CH2 < IC_threshold) || (delta_clk_TIM8_CH2 < IC_threshold))
            RPM_CH2 = RPM_res_per_tick*((float)delta_encoder_CH2);
        
        
        //---------------------------------
        //  Channel 3 RPM
        //---------------------------------
        
        //Timeout for CH3
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH3 = getDeltaClkEncoder(3,1);
        uint16_t delta_clk_TIM8_CH3 = getDeltaClkEncoder(3,2);
        int16_t encoder_count_CH3 = getEncoderCount(3);
        int16_t delta_encoder_CH3 = encoder_count_CH3 - prev_encoder_count_CH3;
        if(delta_encoder_CH3 == 0){
            timeout_CH3++;
            if(timeout_CH3 == timeout_compare_value){
                clearDeltaClkEncoder(3,1);  //Encoder 3 Channel A
                clearDeltaClkEncoder(3,2);  //Encoder 3 Channel B
                timeout_CH3 = 0;
            }
        }
        else
            timeout_CH3 = 0;
        prev_encoder_count_CH3 = encoder_count_CH3;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        //Timer 2 - Channel 3
        if(delta_clk_TIM1_CH3 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH3 = RPM_res_per_IC/(float)delta_clk_TIM1_CH3;
        }
        //Timer 3 - Channel 3
        else if(delta_clk_TIM8_CH3 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH3 = -RPM_res_per_IC/(float)delta_clk_TIM8_CH3;
        }
        else if((delta_clk_TIM1_CH3 < IC_threshold) || (delta_clk_TIM8_CH3 < IC_threshold))
            RPM_CH3 = RPM_res_per_tick*((float)delta_encoder_CH3);
        
        
        //---------------------------------
        //  Channel 4 RPM
        //---------------------------------
        
        //Timeout for CH4
        //Checks Encoder Count
        //if there is not change in encoder count in a 1 sec interval, sets RPM to zero
        //timeout = 1/(64Mhz / 16000 Prescaler / 20 Counter Period)
        //timeout = 0.005sec (200Hz)
        uint16_t delta_clk_TIM1_CH4 = getDeltaClkEncoder(4,1);
        uint16_t delta_clk_TIM8_CH4 = getDeltaClkEncoder(4,2);
        int16_t encoder_count_CH4 = getEncoderCount(4);
        int16_t delta_encoder_CH4 = encoder_count_CH4 - prev_encoder_count_CH4;
        if(delta_encoder_CH4 == 0){
            timeout_CH4++;
            if(timeout_CH4 == timeout_compare_value){
                clearDeltaClkEncoder(4,1);  //Encoder 4, Channel A
                clearDeltaClkEncoder(4,2);  //Encoder 4, Channel B
                timeout_CH4 = 0;
            }
        }
        else
            timeout_CH4 = 0;
        prev_encoder_count_CH4 = encoder_count_CH4;
        
        //Max velocity for Delta Time
        //Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
        //320 = 64Mhz / 1000 / 200Hz
        if(delta_clk_TIM1_CH4 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH4 = RPM_res_per_IC/(float)delta_clk_TIM1_CH4;
        }
        else if(delta_clk_TIM8_CH4 >= IC_threshold){
            //RPM = 60Sec / ( 64Enc * Delta_time)
            //Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
            RPM_CH4 = -RPM_res_per_IC/(float)delta_clk_TIM8_CH4;
        }
        else if((delta_clk_TIM1_CH4 < IC_threshold) || (delta_clk_TIM8_CH4 < IC_threshold))
            RPM_CH4 = RPM_res_per_tick*((float)delta_encoder_CH4);
    }
}