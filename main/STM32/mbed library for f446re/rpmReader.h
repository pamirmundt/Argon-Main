/**
  ******************************************************************************
  * @file    rpmReader.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RPMREADER_H
#define __RPMREADER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "encoderReader.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

//Get Motor RPM
float getMotorRPM(uint8_t motorNumber);

#ifdef __cplusplus
}
#endif

#endif /* __RPMREADER_H */
