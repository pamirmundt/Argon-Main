/**
  ******************************************************************************
  * @file    base.h
  * @brief   Something here
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BASE_H
#define __BASE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "Inits.h"
#include "rpmReader.h"
#include "motorControl.h"
/* Exported types ------------------------------------------------------------*/

enum mecanumWheels {
    noWheel,
    frontLeft,      //WheelNumber 1
    frontRight,     //WheelNumber 2
    rearLeft,       //WheelNumber 3
    rearRight,      //WheelNumber 4
};


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void baseInit(void);
void baseEncoderStart(void);
void baseRpmStart(void);
void basePwmStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_H */
