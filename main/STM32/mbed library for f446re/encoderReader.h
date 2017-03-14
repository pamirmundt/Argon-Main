/**
  ******************************************************************************
  * @file    encoderReader.h
  * @brief   Something here
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODERREADER_H
#define __ENCODERREADER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int16_t get_encoder1_count(void);
int16_t get_encoder2_count(void);
int16_t get_encoder3_count(void);
int16_t get_encoder4_count(void);

uint16_t get_delta_clk_TIM1_CH1(void);
uint16_t get_delta_clk_TIM1_CH2(void);
uint16_t get_delta_clk_TIM1_CH3(void);
uint16_t get_delta_clk_TIM1_CH4(void);

uint16_t get_delta_clk_TIM8_CH1(void);
uint16_t get_delta_clk_TIM8_CH2(void);
uint16_t get_delta_clk_TIM8_CH3(void);
uint16_t get_delta_clk_TIM8_CH4(void);

void clear_delta_clk_TIM1_CH1(void);
void clear_delta_clk_TIM1_CH2(void);
void clear_delta_clk_TIM1_CH3(void);
void clear_delta_clk_TIM1_CH4(void);

void clear_delta_clk_TIM8_CH1(void);
void clear_delta_clk_TIM8_CH2(void);
void clear_delta_clk_TIM8_CH3(void);
void clear_delta_clk_TIM8_CH4(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODERREADER_H */
