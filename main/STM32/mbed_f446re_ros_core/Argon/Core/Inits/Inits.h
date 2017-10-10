/**
  ******************************************************************************
  * @file    Inits.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INITS_H
#define __INITS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private define ------------------------------------------------------------*/
#define motorPWM1_Pin GPIO_PIN_0
#define motorPWM1_GPIO_Port GPIOA
#define MotorPWM2_Pin GPIO_PIN_1
#define MotorPWM2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define motorEncoderA2_Pin GPIO_PIN_6
#define motorEncoderA2_GPIO_Port GPIOA
#define motorEncoderB2_Pin GPIO_PIN_7
#define motorEncoderB2_GPIO_Port GPIOA
#define MotorPWM3_Pin GPIO_PIN_2
#define MotorPWM3_GPIO_Port GPIOB
#define MotorPWM4_Pin GPIO_PIN_10
#define MotorPWM4_GPIO_Port GPIOB
#define motorDirection2_Pin GPIO_PIN_14
#define motorDirection2_GPIO_Port GPIOB
#define motorDirection3_Pin GPIO_PIN_15
#define motorDirection3_GPIO_Port GPIOB
#define motorEncoderA4_Pin GPIO_PIN_6
#define motorEncoderA4_GPIO_Port GPIOC
#define motorEncoderB4_Pin GPIO_PIN_7
#define motorEncoderB4_GPIO_Port GPIOC
#define motorDirection4_Pin GPIO_PIN_9
#define motorDirection4_GPIO_Port GPIOC
#define motorEncoderA1_Pin GPIO_PIN_8
#define motorEncoderA1_GPIO_Port GPIOA
#define motorEncoderB1_Pin GPIO_PIN_9
#define motorEncoderB1_GPIO_Port GPIOA
#define motorDirection1_Pin GPIO_PIN_10
#define motorDirection1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define motorEncoderA3_Pin GPIO_PIN_6
#define motorEncoderA3_GPIO_Port GPIOB
#define motorEncoderB3_Pin GPIO_PIN_7
#define motorEncoderB3_GPIO_Port GPIOB

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_TIM2_Init(void);
void MX_TIM6_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H */
