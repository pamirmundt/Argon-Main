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

#include "motor.h"
#include "base.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
void calcPID(void);

//I2C RX/TX Buffer
const uint16_t RXBUFFERSIZE = 12;		//Receive buffer size
const uint16_t TXBUFFERSIZE = 13;		//Transmit buffer size
uint8_t txBuffer[TXBUFFERSIZE];
uint8_t rxBuffer[RXBUFFERSIZE];

//Base and motor
//Base MAX SPEED = 0.97m/s
Motor motorFL, motorFR, motorRL, motorRR;
Base mecanumBase;

//Slave wheel I2C Addresses
const uint16_t FRwheelAddr = 0x40;
const uint16_t FLwheelAddr = 0x42;
const uint16_t RRwheelAddr = 0x44;
const uint16_t RLwheelAddr = 0x46;

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

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void baseInit(void);

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
  MX_USART2_UART_Init();
  MX_TIM9_Init();

  /* USER CODE BEGIN 2 */
	
	//Initialize Base
	baseInit();
	
	//Control Interrupt Init
	//Timer 9 - 90MHz
	//180Mhz/60000/30 = 100Hz
	HAL_TIM_Base_Start_IT(&htim9);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Update Base Position
		base_getPosition(&mecanumBase, &mecanumBase.longitudinalPosition, &mecanumBase.transversalPosition, &mecanumBase.orientation);
		
		
		//Set wheel velocity(RPM)
		motor_setRPM(motorFL, mecanumBase.wheelTorques[0]*Ktv);
		motor_setRPM(motorFR, mecanumBase.wheelTorques[1]*Ktv);
		motor_setRPM(motorRL, mecanumBase.wheelTorques[2]*Ktv);
		motor_setRPM(motorRR, mecanumBase.wheelTorques[3]*Ktv);
		
		
		//DEBUG ONLY
		//base_setVelocity(mecanumBase, 0.01f, 0.0f, 0.0f);
		//base_getVelocity(mecanumBase, &longitudinalVelocity, &transversalVelocity, &angularVelocity);
		
		//Serial Print
		char str[60];
		int test1 = mecanumBase.controlLong*1000;
		int test2 = mecanumBase.refLongitudinalPosition*1000;
		int test3 = mecanumBase.errLong*1000;
		//sprintf(str,"$%d %d %d;", test1, test2, test3);
		sprintf(str,"%f %f %f \n", mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation);
		HAL_UART_Transmit_IT(&huart2,(uint8_t*)str, strlen(str));
		
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

/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 60000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 29;
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

/**
  * @brief  Initialize Base
							- Reset wheels
							-	Reset base parameters
  * @param  None
  * @retval None
  */
void baseInit(){
	//Reset wheel instances
	memset(&motorFL, 0, sizeof(motorFL));
	memset(&motorFR, 0, sizeof(motorFR));
	memset(&motorRL, 0, sizeof(motorRL));
	memset(&motorRR, 0, sizeof(motorRR));
	
	//Reset base instance
	memset(&mecanumBase, 0, sizeof(mecanumBase));
	
	//Init motors	i2c addresses
	motorFL.slaveAddr = FLwheelAddr;
	motorFR.slaveAddr = FRwheelAddr;
	motorRL.slaveAddr = RLwheelAddr;
	motorRR.slaveAddr = RRwheelAddr;
	
	//Set motors
	mecanumBase.frontLeftWheel = motorFL;
	mecanumBase.frontRightWheel = motorFR;
	mecanumBase.rearLeftWheel = motorRL;
	mecanumBase.rearRightWheel = motorRR;
	
	//Reset Wheels
	motor_reset(motorFL);
	motor_reset(motorFR);
	motor_reset(motorRL);
	motor_reset(motorRR);
	
	//Set PID constants
	//Longitudinal PID Constants
	mecanumBase.KLp = KLp;
	mecanumBase.KLi = KLi;
	mecanumBase.KLd = KLd;
	
	//Transversal PID Constants
	mecanumBase.KTp = KTp;
	mecanumBase.KTi = KTi;
	mecanumBase.KTd = KTd;
	
	//Orientation PID Constants
	mecanumBase.KOp = KOp;
	mecanumBase.KOi = KOi;
	mecanumBase.KOd = KOd;
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
