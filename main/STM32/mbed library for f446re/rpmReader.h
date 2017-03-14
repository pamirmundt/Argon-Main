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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

float get_motor1_RPM(void);
float get_motor2_RPM(void);
float get_motor3_RPM(void);
float get_motor4_RPM(void);


#ifdef __cplusplus
}
#endif

#endif /* __RPMREADER_H */
