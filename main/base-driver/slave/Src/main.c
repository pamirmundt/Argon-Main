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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "motor.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//I2C Configurations
//This device is a SLAVE
#define I2C_ADDRESS      	0x40
#define notReady					0x00			//Used for I2C ready
#define ready							0x01			//Used for I2C not ready

//I2C RX/TX Buffer
const uint16_t RXBUFFERSIZE = 13;		//Receive buffer size
const uint16_t TXBUFFERSIZE = 12;		//Transmit buffer size
uint8_t txBuffer[TXBUFFERSIZE];
uint8_t rxBuffer[RXBUFFERSIZE];

//I2C Ready Check
volatile uint8_t i2cReady = ready;

//**********************************************
//Motor Control Configurations
//**********************************************

//Timer Configurations
	//Timer1 (128Mhz) - PWM
	//Timer2 (64Mhz)  - Encoder Mode (4x)
	//Timer3 (64Mhz)  - Interrupt - 200Hz

//Nucleo F303K8 Pin Configurations
	//A0 - Encoder A
	//A1 - Encoder B
	//A2 - AIN1
	//A3 - AIN2
	//A4 - STBY
	//D9 - PWM
	
//Motor: JGA25-271
//Encoder Resolution: 334 pulse/rotation
//Gear Ratio: 13.552
//Create Motor
Motor m;
const float encoderRes = 334.0f;	//Encoder Resolution 334 pulse/rotation
const float gearRatio = 13.552f;  //Gear Ratio 1:13.552

//Control Period / Frequency (RPM and PID Calculation, Motor PWM)
const float contPeriod = 1.0f/200.0f;

//Default Kp, Ki, Kd
float Kp = 0.15f;
float Ki = 3.5f;
float Kd = 0.0f;

//PWM Resolution (DO NOT CHANGE THIS VALUE)
//	12Bit Resolution: 0-4095
uint16_t PWM_resolution = 4095;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void calcRPM(void);
void calcPID(void);
void setMotorPWM(void);
void motorInit(void);

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	//Initialize Motor
	//Enable Motor
	motorInit();
	
	//Enable I2C interrupt
	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
	
	//PWM Init
	//Timer1 - 128Mhz
	//128Mhz/4096 = 31,250Hz
	//12bit Resolution 0-4095
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	//Encoder Init
	//Timer2 - 64Mhz
	//4X Encoding
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	
	//Control Interrupt Init
	//Timer3 - 64Mhz
	//64Mhz/64000/5 = 200Hz
	HAL_TIM_Base_Start_IT(&htim3);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Print RPM
		char msg[20];
		sprintf(msg, "%f \r\n", (m.RPM/gearRatio));
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = I2C_ADDRESS;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_resolution;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Calculates RPM. Count the encoder tick every 'contPeriod' (200Hz - 0.005sec)
	*							count_per_minute = (1min/0.005sec)*countDifference
	*							4xRPM = count_per_minute / encoderResolution
	*							RPM = 4xRPM/4 (4x Encoding)
  * @param  None
  * @retval None
  */
void calcRPM(){
	m.encPos = __HAL_TIM_GET_COUNTER(&htim2);		//Get Count
	m.dPos = m.encPos - m.encPrevPos; 					//Delta position (difference)
  m.encPrevPos = m.encPos;
  m.RPM = (((60.0f)/(contPeriod*encoderRes))*((float)m.dPos))/4.0f;
}

/**
  * @brief  Calculates PID. Calculate PID every 'contPeriod' (200Hz - 0.005sec)
  * @param  None
  * @retval None
  */
void calcPID(){
	m.errRPM = m.refRPM*gearRatio - m.RPM;
  m.integral = m.integral + m.errRPM*contPeriod;
	//Velocity Control -> PI Control
  //m.derivative = (m.errRPM - m.errPrevRPM)/contPeriod;
  m.control = Kp*m.errRPM + Ki*m.integral;//+ Kd*m.derivative;
  m.errPrevRPM = m.errRPM;
}

/**
  * @brief  Sets Motor PWM. Saturate PWM and change direction (TB6612FNG)
	*								AIN1 | AIN2 | 	Output
	*									H	 |   H	| Short Brake
	*									L	 |	 H	|			CCW
	*									H	 |	 L	|			CW
	*									L	 |	 L	|			STOP
  * @param  None
  * @retval None
  */
void setMotorPWM(){
	m.PWM = m.control;
	//Saturation
	if(m.PWM < -PWM_resolution)
		m.PWM = -PWM_resolution;
	else if(m.PWM > PWM_resolution)
		m.PWM = PWM_resolution;

  if(m.PWM == 0){
		//Brake: HIGH - HIGH
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);			//A2 - AIN1
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);			//A3 - AIN2
  }
  if(m.PWM > 0){
		//CW Rotation: HIGH - LOW
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);			//A2 - AIN1
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);		//A3 - AIN2
  } else {
    //CCW Rotation: LOW - HIGH
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);		//A2 - AIN1
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);			//A3 - AIN2
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fabsf(m.PWM));
}

/**
  * @brief  Initialize Motor. Disable STANDBY mode, enable Velocity control mode and set default PID constants
  * @param  None
  * @retval None
  */
void motorInit(){
	//Enable STBY Pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	
	//Set Default Values
	m.Kp = Kp;
	m.Ki = Ki;
	m.Kd = Kd;
	
	//Set Default Mode as Velocity PID
	//Manuel Mode: 				0x00
	//Velocity PID Mode: 	0x01
	m.mode = 0x01;
}

/**
  * @brief  Timer 3 Interrupt at 200Hz. Calculate RPM, calculate PID and drive motor
  * @param  htim pointer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// This callback is automatically called by the HAL on the UEV event
	if(htim->Instance == TIM3){
		calcRPM();
		
		switch(m.mode){
			//Manuel Mode
			case 0x00: {
				m.control = m.PWM;
			} break;
			
			//Velocity PID Mode:
			case 0x01: {
				calcPID();
			} break;
		}
		
		setMotorPWM();
	}
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	//End Of Slave Receive - I2C Bus is Ready
	i2cReady = ready;
	cmdParser();

	//Restart I2C bus for the next interrupt
	__HAL_I2C_DISABLE(I2cHandle);
	__HAL_I2C_ENABLE(I2cHandle);
	
	//Enable Interrupt
	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	//End Of Slave Transmit - I2C Bus is Ready
	i2cReady = ready;

	//Restart I2C bus for the next interrupt
	__HAL_I2C_DISABLE(I2cHandle);
	__HAL_I2C_ENABLE(I2cHandle);
	
	//Enable Interrupt
	__HAL_I2C_ENABLE_IT(I2cHandle,I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_TXI | I2C_IT_RXI);
}

/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXT line 23.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

	//Check if there is incoming data && I2C bus is ready
	if(i2cReady == ready && __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_DIR) == 0 && HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY){
		//Start receveing - I2C Bus is not ready
		i2cReady = notReady;
		//Disable Interrupt to avoid the risk of I2C interrupt handle execution before current process
		__HAL_I2C_DISABLE_IT(&hi2c1,I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_TXI | I2C_IT_RXI);
		//Receive the data
		HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rxBuffer, RXBUFFERSIZE);
	}
	
	//Check if there is outgoing data && I2C bus is ready
	if(i2cReady == ready && __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_DIR) == 1 && HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY){
		//Start transmitting - I2C Bus is not ready
		i2cReady = notReady;	
		//Disable Interrupt to avoid the risk of I2C interrupt handle execution before current process
		__HAL_I2C_DISABLE_IT(&hi2c1,I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_TXI | I2C_IT_RXI);
		//Transmit the data
		HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)&txBuffer, TXBUFFERSIZE);
	}

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
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
