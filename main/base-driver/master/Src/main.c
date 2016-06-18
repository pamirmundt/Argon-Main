/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>

#include "base.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted. */
#define FLAG_TIMEOUT ((int)0x1000)
#define LONG_TIMEOUT ((int)0x8000)

//Timers
//Timer 9 - 90MHz
//180Mhz/60000/30 = 100Hz

//Base and motor
//Base MAX SPEED = 0.97m/s
Motor motorFL, motorFR, motorRL, motorRR;
Base mecanumBase, initBase;

//Slave wheel I2C Addresses
const uint16_t FLwheelAddr = 0x40;
const uint16_t FRwheelAddr = 0x42;
const uint16_t RLwheelAddr = 0x44;
const uint16_t RRwheelAddr = 0x46;

//Control configurations
//100Hz PID postion control
const float contPeriod = 1.0f/100.0f;

//PID Constants
//Longitude position control constants
const float KLp = 13.5f, KLi = 0.0f, KLd = 0.01f; 
//Transversal position control constants
const float KTp = 13.5f, KTi = 0.0f, KTd = 0.01f;
//Angular(orientation) position control constants
const float KOp = 0.6f, KOi = 0.0f, KOd = 0.0001f;

//Torque to velocity constant
const float Ktv = 100000.0f;

	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void baseInit(void);
void calcPID(void);
int read(char *data, int length);
int write(const char *data, int length);
int i2c_slave_read(I2C_HandleTypeDef *I2cHandle, char *data, int length);
int i2c_slave_write(I2C_HandleTypeDef *I2cHandle, const char *data, int length);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  //MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
	
	//Initialize Base and wheel addresses
	motorFL.slaveAddr = FLwheelAddr;
	motorFR.slaveAddr = FRwheelAddr;
	motorRL.slaveAddr = RLwheelAddr;
	motorRR.slaveAddr = RRwheelAddr;

	base_init(&mecanumBase, &motorFL, &motorFR, &motorRL, &motorRR);
	
	//Default Control Mode: Velocity Control (0x01)
	base_setControlMode(&mecanumBase, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//I2C-2 Receive/Transmit	
		int hi2c2State = hi2c2_get_state();
		
		switch (hi2c2State) {
			case ReadAddressed: {
				write(hi2c2TXBuffer, hi2c2TXBUFFERSIZE);
				while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
			} break;
			
			case WriteAddressed : {
				//Check for the first byte if RPI-master is writing or reading
				/*	Issue: Raspberry Pi smbus-protocol is always sending a "register" byte
										at the start of the buffer before reading/writing. So it has to
										be dumped or used. In this case it has been used to understand 
										if the master is going to write or read.
													1.Byte -> 0:masterWriting, 1:masterReading
				*/
				
				//Read first byte to temporarity buffer (char array)
				char temp[1];
				read(temp, 1);
				while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
				
				//First byte of message
				uint8_t reg;
				memcpy(&reg, &temp[0], sizeof(reg));
				
				//If master is writing
				if(reg == 0x00){
					read(hi2c2RXBuffer, (hi2c2RXBUFFERSIZE));
					while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
					
					slaveCmdParser();
				}
				
			} break;
		}
		
		//Update Base Position
		base_getUpdatePosition(&mecanumBase, &mecanumBase.longitudinalPosition, &mecanumBase.transversalPosition, &mecanumBase.orientation);
			
		
		//Position Control Mode
		if(mecanumBase.controlMode == 0x02){
			//Set wheel velocity(RPM)
			motor_setRPM(motorFL, mecanumBase.wheelTorques[0]*Ktv);
			motor_setRPM(motorFR, mecanumBase.wheelTorques[1]*Ktv);
			motor_setRPM(motorRL, mecanumBase.wheelTorques[2]*Ktv);
			motor_setRPM(motorRR, mecanumBase.wheelTorques[3]*Ktv);
		}
		
		//DEBUG ONLY
		//base_setVelocity(mecanumBase, 0.0f, 0.0f, 0.1f);
		//base_getVelocity(mecanumBase, &longitudinalVelocity, &transversalVelocity, &angularVelocity);
		
		//Serial Print
		//char str[60];
		//int test1 = mecanumBase.controlLong*1000;
		//int test2 = mecanumBase.refLongitudinalPosition*1000;
		//int test3 = mecanumBase.errLong*1000;
		//sprintf(str,"$%d %d %d;", test1, test2, test3);
		//sprintf(str,"%f %f %f \n", mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation);
		//HAL_UART_Transmit_IT(&huart2,(uint8_t*)str, strlen(str));
		
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0x40;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
	
	/* Enable Address Acknowledge */
	hi2c2.Instance->CR1 |= I2C_CR1_ACK;
}

/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 60000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 14;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

int read(char *data, int length) {
    return i2c_slave_read(&hi2c2, data, length) != length;
}

int i2c_slave_read(I2C_HandleTypeDef *I2cHandle, char *data, int length)
{
    uint32_t Timeout;
    int size = 0;

    while (length > 0) {
        // Wait until RXNE flag is set
        // Wait until the byte is received
        Timeout = FLAG_TIMEOUT;
        while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_RXNE) == RESET) {
            Timeout--;
            if (Timeout == 0) {
                return -1;
            }
        }

        // Read data from DR
        (*data++) = I2cHandle->Instance->DR;
        length--;
        size++;

        if ((__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_BTF) == SET) && (length != 0)) {
            // Read data from DR
            (*data++) = I2cHandle->Instance->DR;
            length--;
            size++;
        }
    }

    // Wait until STOP flag is set
    Timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_STOPF) == RESET) {
        Timeout--;
        if (Timeout == 0) {
            return -1;
        }
    }

    // Clear STOP flag
    __HAL_I2C_CLEAR_STOPFLAG(I2cHandle);

    // Wait until BUSY flag is reset 
    Timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_BUSY) == SET) {
        Timeout--;
        if (Timeout == 0) {
            return -1;
        }
    }

    return size;
}

int write(const char *data, int length) {
    return i2c_slave_write(&hi2c2, data, length) != length;
}

int i2c_slave_write(I2C_HandleTypeDef *I2cHandle, const char *data, int length)
{
    uint32_t Timeout;
    int size = 0;

    while (length > 0) {
        /* Wait until TXE flag is set */
        Timeout = FLAG_TIMEOUT;
        while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_TXE) == RESET) {
            Timeout--;
            if (Timeout == 0) {
                return -1;
            }
        }


        /* Write data to DR */
        I2cHandle->Instance->DR = (*data++);
        length--;
        size++;

        if ((__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_BTF) == SET) && (length != 0)) {
            /* Write data to DR */
            I2cHandle->Instance->DR = (*data++);
            length--;
            size++;
        }
    }

    /* Wait until AF flag is set */
    Timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_AF) == RESET) {
        Timeout--;
        if (Timeout == 0) {
            return -1;
        }
    }


    /* Clear AF flag */
    __HAL_I2C_CLEAR_FLAG(I2cHandle, I2C_FLAG_AF);


    /* Wait until BUSY flag is reset */
    Timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(I2cHandle, I2C_FLAG_BUSY) == SET) {
        Timeout--;
        if (Timeout == 0) {
            return -1;
        }
    }

    I2cHandle->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(I2cHandle);

    return size;
}

/**
  * @brief  Timer 9 Interrupt at 100Hz
  * @param  htim pointer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// This callback is automatically called by the HAL on the UEV event
	if(htim->Instance == TIM9){
		calcPID();
		base_calcWheelTorques(&mecanumBase, mecanumBase.wheelTorques);
	}
	
}

/**
  * @brief  Calculates PID. Calculate PID every 'contPeriod' (100Hz - 0.01sec)
	*					PID Position control input: base position error (q_x, q_y, q_theta)
	*					PID Position control output: base force (F_x, F_y, F_theta)
  * @param  None
  * @retval None
  */
void calcPID(){
	
	//Base Longitudinal Position PD
	mecanumBase.errLong = mecanumBase.refLongitudinalPosition - mecanumBase.longitudinalPosition;
	//mecanumBase.integralLong = mecanumBase.integralLong + mecanumBase.errLong * contPeriod;
	mecanumBase.derivativeLong = (mecanumBase.errLong - mecanumBase.errPrevLong) / contPeriod;
	mecanumBase.controlLong = mecanumBase.KLp * mecanumBase.errLong + mecanumBase.KLd * mecanumBase.derivativeLong; //+ mecanumBase.Ki * mecanumBase.integralLong;
	mecanumBase.errPrevLong = mecanumBase.errLong;
	
	
	//Base Transversal Position PD
	mecanumBase.errTrans = mecanumBase.refTransversalPosition - mecanumBase.transversalPosition;
	//mecanumBase.integralTrans = mecanumBase.integralTrans + mecanumBase.errTrans * contPeriod;
	mecanumBase.derivativeTrans = (mecanumBase.errTrans - mecanumBase.errPrevTrans) / contPeriod;
	mecanumBase.controlTrans = mecanumBase.KTp * mecanumBase.errPrevTrans + mecanumBase.KTd * mecanumBase.derivativeTrans; //+ mecanumBase.Ki * mecanumBase.integralTrans;
	mecanumBase.errPrevTrans = mecanumBase.errTrans;
	
	//Base Orientation PD
	mecanumBase.errOrien = mecanumBase.refOrientation - mecanumBase.orientation;
	//mecanumBase.integralOrien = mecanumBase.integralOrien + mecanumBase.errOrien * contPeriod;
	mecanumBase.derivativeOrien = (mecanumBase.errOrien - mecanumBase.errPrevOrien) / contPeriod;
	mecanumBase.controlOrien = mecanumBase.KOp * mecanumBase.errOrien + mecanumBase.KOd * mecanumBase.derivativeOrien; //+ mecanumBase.KOi * mecanumBase.integralOrien;
	mecanumBase.errPrevOrien = mecanumBase.errOrien;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
