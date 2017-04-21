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
int16_t getEncoderCount(uint8_t motorNumber);
uint16_t getDeltaClkEncoder(uint8_t motorNumber, uint8_t channelNumber);
void clearDeltaClkEncoder(uint8_t motorNumber, uint8_t channelNumber);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODERREADER_H */
