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

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM8_Init(void);
void MX_TIM9_Init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H */
