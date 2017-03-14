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
#include "rpmReader.h"
#include "Inits.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void base_init(void);
void base_encoder_start(void);
void base_RPM_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_H */
