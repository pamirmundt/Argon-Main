/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint16_t prev_capture_TIM2_CH1 = 0;
volatile uint16_t prev_capture_TIM2_CH2 = 0;
volatile uint16_t prev_capture_TIM2_CH3 = 0;
volatile uint16_t prev_capture_TIM2_CH4 = 0;
volatile uint16_t delta_clk_TIM2_CH1 = 0;
volatile uint16_t delta_clk_TIM2_CH2 = 0;
volatile uint16_t delta_clk_TIM2_CH3 = 0;
volatile uint16_t delta_clk_TIM2_CH4 = 0;

volatile uint16_t prev_capture_TIM3_CH1 = 0;
volatile uint16_t prev_capture_TIM3_CH2 = 0;
volatile uint16_t prev_capture_TIM3_CH3 = 0;
volatile uint16_t prev_capture_TIM3_CH4 = 0;
volatile uint16_t delta_clk_TIM3_CH1 = 0;
volatile uint16_t delta_clk_TIM3_CH2 = 0;
volatile uint16_t delta_clk_TIM3_CH3 = 0;
volatile uint16_t delta_clk_TIM3_CH4 = 0;

volatile int16_t encoder_count_CH1 = 0;
volatile int16_t encoder_count_CH2 = 0;
volatile int16_t encoder_count_CH3 = 0;
volatile int16_t encoder_count_CH4 = 0;

volatile int16_t prev_encoder_count_CH1 = 0;
volatile int16_t prev_encoder_count_CH2 = 0;
volatile int16_t prev_encoder_count_CH3 = 0;
volatile int16_t prev_encoder_count_CH4 = 0;

volatile float RPM_CH1 = 0.0f;
volatile float RPM_CH2 = 0.0f;
volatile float RPM_CH3 = 0.0f;
volatile float RPM_CH4 = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 512);
	
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1024);
	
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
	
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000);


	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start_IT(&htim15);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
		//GPIOF->BSRR = GPIO_BSRR_BS_1;
		//GPIOF->BSRR = GPIO_BSRR_BR_1;	
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 15999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	
	if(htim->Instance == TIM2){
		
		//Timer 2 - Channel 1
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			encoder_count_CH1++;
			
			uint16_t input_capture_TIM2_CH1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);	//read TIM2 channel 1 capture value
			delta_clk_TIM2_CH1 = input_capture_TIM2_CH1 - prev_capture_TIM2_CH1;
			prev_capture_TIM2_CH1 = input_capture_TIM2_CH1;
		}
		
		//Timer 2 - Channel 2
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			encoder_count_CH2++;
			
			uint16_t input_capture_TIM2_CH2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);	//read TIM2 channel 2 capture value
			delta_clk_TIM2_CH2 = input_capture_TIM2_CH2 - prev_capture_TIM2_CH2;
			prev_capture_TIM2_CH2 = input_capture_TIM2_CH2;
		}
		
		//Timer 2 - Channel 3
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			encoder_count_CH3++;
			
			uint16_t input_capture_TIM2_CH3 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);	//read TIM2 channel 3 capture value
			delta_clk_TIM2_CH3 = input_capture_TIM2_CH3 - prev_capture_TIM2_CH3;
			prev_capture_TIM2_CH3 = input_capture_TIM2_CH3;
		}
		
		//Timer 2 - Channel 4
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			encoder_count_CH4++;
			
			uint16_t input_capture_TIM2_CH4 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);	//read TIM2 channel 4 capture value
			delta_clk_TIM2_CH4 = input_capture_TIM2_CH4 - prev_capture_TIM2_CH4;
			prev_capture_TIM2_CH4 = input_capture_TIM2_CH4;
		}
	}
	
	else if(htim->Instance == TIM3){
		//Timer 3 - Channel 1
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			encoder_count_CH1--;
			
			uint16_t input_capture_TIM3_CH1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);	//read TIM3 channel 1 capture value
			delta_clk_TIM3_CH1 = input_capture_TIM3_CH1 - prev_capture_TIM3_CH1;
			prev_capture_TIM3_CH1 = input_capture_TIM3_CH1;
			
		}
		
		//Timer 3 - Channel 2
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			encoder_count_CH2--;
			
			uint16_t input_capture_TIM3_CH2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);	//read TIM3 channel 2 capture value
			delta_clk_TIM3_CH2 = input_capture_TIM3_CH2 - prev_capture_TIM3_CH2;
			prev_capture_TIM3_CH2 = input_capture_TIM3_CH2;
		}
		
		//Timer 3 - Channel 3
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			encoder_count_CH3--;
			
			uint16_t input_capture_TIM3_CH3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);	//read TIM3 channel 3 capture value
			delta_clk_TIM3_CH3 = input_capture_TIM3_CH3 - prev_capture_TIM3_CH3;
			prev_capture_TIM3_CH3 = input_capture_TIM3_CH3;
		}
		
		//Timer 3 - Channel 4
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			encoder_count_CH4--;
			
			uint16_t input_capture_TIM3_CH4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);	//read TIM3 channel 3 capture value
			delta_clk_TIM3_CH4 = input_capture_TIM3_CH4 - prev_capture_TIM3_CH4;
			prev_capture_TIM3_CH4 = input_capture_TIM3_CH4;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance == TIM15)
	{
		//---------------------------------
		//	Channel 1 RPM
		//---------------------------------
		int16_t delta_encoder_CH1 = encoder_count_CH1 - prev_encoder_count_CH1;
		prev_encoder_count_CH1 = encoder_count_CH1;
		
		//Max velocity for Delta Time
		//Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
		//320 = 64Mhz / 1000 / 200Hz
		//Timer 2 - Channel 1
		if(delta_clk_TIM2_CH1 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH1 = 60000.0f/(float)delta_clk_TIM2_CH1;	
		}
		//Timer 3 - Channel 1
		else if(delta_clk_TIM3_CH1 >= 320){
				//RPM = 60Sec / ( 64Enc * Delta_time)
				//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
				RPM_CH1 = -60000.0f/(float)delta_clk_TIM3_CH1;
		}
		else if((delta_clk_TIM2_CH1 < 320) && (delta_clk_TIM3_CH1 < 320))
			RPM_CH1 = 187.5f*((float)delta_encoder_CH1);
		
		
		//---------------------------------
		//	Channel 2 RPM
		//---------------------------------
		int16_t delta_encoder_CH2 = encoder_count_CH2 - prev_encoder_count_CH2;
		prev_encoder_count_CH2 = encoder_count_CH2;
		
		//Max velocity for Delta Time
		//Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
		//320 = 64Mhz / 1000 / 200Hz
		//Timer 2 - Channel 2
		if(delta_clk_TIM2_CH2 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH2 = 60000.0f/(float)delta_clk_TIM2_CH2;
		}
		//Timer 3 - Channes 3
		else if(delta_clk_TIM3_CH2 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH2 = -60000.0f/(float)delta_clk_TIM3_CH2;
		}
		else if((delta_clk_TIM2_CH2 < 320) && (delta_clk_TIM3_CH2 < 320))
			RPM_CH2 = 187.5f*((float)delta_encoder_CH2);
		
		//---------------------------------
		//	Channel 3 RPM
		//---------------------------------
		int16_t delta_encoder_CH3 = encoder_count_CH3 - prev_encoder_count_CH3;
		prev_encoder_count_CH3 = encoder_count_CH3;
		
		//Max velocity for Delta Time
		//Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
		//320 = 64Mhz / 1000 / 200Hz
		//Timer 2 - Channel 3
		if(delta_clk_TIM2_CH3 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH3 = 60000.0f/(float)delta_clk_TIM2_CH3;
		}
		//Timer 3 - Channel 3
		else if(delta_clk_TIM3_CH3 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH3 = -60000.0f/(float)delta_clk_TIM3_CH3;
		}
		else if((delta_clk_TIM2_CH3 < 320) && (delta_clk_TIM3_CH3 < 320))
			RPM_CH3 = 187.5f*((float)delta_encoder_CH3);
		
		
		//---------------------------------
		//	Channel 4 RPM
		//---------------------------------
		int16_t delta_encoder_CH4 = encoder_count_CH4 - prev_encoder_count_CH4;
		prev_encoder_count_CH4 = encoder_count_CH4;
		
		//Max velocity for Delta Time
		//Count Limit(timer count) = 64Mhz / 1000Prescale / 200Hz (Velocity calculation loop)
		//320 = 64Mhz / 1000 / 200Hz
		if(delta_clk_TIM2_CH4 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH4 = 60000.0f/(float)delta_clk_TIM2_CH4;
		}
		else if(delta_clk_TIM3_CH4 >= 320){
			//RPM = 60Sec / ( 64Enc * Delta_time)
			//Delta_time(seconds) = delta_count / (64Mhz / 1000Prescale)
			RPM_CH4 = -60000.0f/(float)delta_clk_TIM3_CH4;
		}
		else if((delta_clk_TIM2_CH4 < 320) && (delta_clk_TIM3_CH4 < 320))
			RPM_CH4 = 187.5f*((float)delta_encoder_CH4);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
