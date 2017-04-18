/**
  ******************************************************************************
  * @file    motorControl.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void set_motor_pwm(uint32_t motorNumber, uint16_t pwm);
void set_motor_direction(uint8_t motorNumber, uint8_t dir);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORCONTROL_H */
