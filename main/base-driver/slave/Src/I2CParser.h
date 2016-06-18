#ifndef I2CParser_H
#define I2CParser_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "motor.h"

extern TIM_HandleTypeDef htim2;

/** @brief Parse the in coming data from I2C
  * @param  None
  * @retval None
  */
void cmdParser(void);

#endif
