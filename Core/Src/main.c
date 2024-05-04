/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "../../ECUAL/I2C_LCD/I2C_LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MyI2C_LCD I2C_LCD_1		// LCD Stuff
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Global Variables
char msg[23];				// UART Message Buffer 1
char msg2[60];				// UART Message Buffer 2
char msgRx[5] = {0};		// UART receive message buffer
uint16_t potValue;
int uartFlag = 1;			// 1 = Input received
int ledOneFlag = 1;			// 1 = On
int ledTwoFlag = 1;			// 1 = On
int buttonTwoFlag = 1;		// 1 = On
int stateTracker;			// 1 = A, 2 = B, 3 = C

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

// Test functions
void hardwareTestLED(void);												// Test LEDs wiring
void hardwareTestButton(void);											// Test buttons wiring
void hardwareTestLCD(void);												// Test LCD wiring

// Control functions
void getPotValue(void);													// ADC Reader - Print value
int myMap(int x, int in_min, int in_max, int out_min, int out_max);		// Map ADC to PWM
void motorControl(int adcValue);										// Send PWM to motor
int getAdcFromPot();													// ADC Reader - Return value

// State Functions
void stateMachineController(int state);									// Executes states
void stateMachineDecider();											 	// Tracks states
void stateHandlerA(void);
void stateHandlerB(void);
void stateHandlerC(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  // Enable the TIM2 peripheral
  __HAL_RCC_TIM2_CLK_ENABLE();

  // Enable the TIM6 peripheral
  __HAL_RCC_TIM6_CLK_ENABLE();

  // Enable the TIM7 peripheral
  __HAL_RCC_TIM7_CLK_ENABLE();

  // Enable the TIM14 peripheral
   __HAL_RCC_TIM14_CLK_ENABLE();

  // Enable the peripheral IRQ for TIM2
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // Enable the peripheral IRQ for TIM6
  HAL_NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);

  // Enable the peripheral IRQ for TIM7
  HAL_NVIC_SetPriority(TIM7_LPTIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_LPTIM2_IRQn);

  // Enable the peripheral IRQ for TIM14
  HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);

  // Start the timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim14);

  // Enable PWM on TIM3 (for motor control)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // I2C Display
  hardwareTestLCD();

  // Variables
  int servoAngle;
  int adcValue;
  stateTracker = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check UART Transmissions
	  HAL_UART_Receive_IT(&huart2, msgRx, 1);

	  // Get potentiometer value
	  HAL_ADC_Start_IT(&hadc1);
	  getPotValue();

	  // State Machine
	  stateMachineDecider();
	  stateMachineController(stateTracker);

	  // Monitor LEDs
	  if (ledOneFlag == 0 ){
		  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
	  } else if (ledTwoFlag == 0){
		  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 319;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3332;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_INPUT_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_INPUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_INPUT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_INPUT_Pin */
  GPIO_InitStruct.Pin = BUTTON_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_INPUT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

	/* Test LEDs are working  */
	void hardwareTestLED(){

		/* Lights ON */
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
		HAL_Delay(100);

		/* Lights OFF */
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}

	/* Get ADC value from potentiometer */
	void getPotValue(void){

		/* Get pot value */
		HAL_ADC_PollForConversion(&hadc1, 5);
		potValue = HAL_ADC_GetValue(&hadc1);

		/* Debug */
		//sprintf(msg, "potValue: %hu\r\n", potValue);
		//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	/* getADC from Pot */
	int getAdcFromPot(){
		/* Get pot value */
		uint16_t potValue;
		HAL_ADC_PollForConversion(&hadc1, 5);
		potValue = HAL_ADC_GetValue(&hadc1);
		return potValue;
	}

	/* Test pushbutton is working */
	void hardwareTestButton(void){
		char msg[23];
		if (HAL_GPIO_ReadPin(BUTTON_INPUT_GPIO_Port, BUTTON_INPUT_Pin) == 0){
			printf(msg, "BUTTON IS BEING PUSHED");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		} else {
			printf(msg, "NO BUTTON :(");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	 	}
	}

	/* Test LCD is working */
	void hardwareTestLCD(void){
		I2C_LCD_Init(MyI2C_LCD);
		I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
		I2C_LCD_WriteString(MyI2C_LCD, "Hardware Test");
		I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
		I2C_LCD_WriteString(MyI2C_LCD, "Meow");
	}

	/* Motor control */
	void motorControl(int adcValue){
		__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, adcValue); // Interpolate pot values to PWM range
	}

	/* Mapping function */
	 int myMap(int x, int in_min, int in_max, int out_min, int out_max){
		 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	 }

	 /* State Machine Controller */
	 // Description: Determines which state the program will execute.
	 //  Input:		 Integer corresponding to state (1 = A, 2 = B, 3 = C)
	 void stateMachineController(int state){

		 switch(state){

		 	 // State A
			 case 1:

				 // Debug message
				 //sprintf(msg, "Executing A.\n\r");
				 //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				 // LEDs go back to normal when not in State B
				 ledOneFlag = 1;
				 ledTwoFlag = 1;

				 stateHandlerA();
				 break;

			 // State B
			 case 2:

				 // Debug message
				 //sprintf(msg, "Executing B.\n\r");
				 //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				 stateHandlerB();
				 break;

			// State C
			 case 3:

				 // Debug message
				 sprintf(msg, "Executing C.\n\r");
				 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				 // LEDs go back to normal when not in State B
				 ledOneFlag = 1;
				 ledTwoFlag = 1;

				 stateHandlerC();
				 break;
		 }
	 }

	 /* State Machine Controller */
	 // Description: Monitors button pushes to update current state value.
	 void stateMachineDecider(){
		 // Go to State B if it is State A
		 if (HAL_GPIO_ReadPin(BUTTON_INPUT_GPIO_Port, BUTTON_INPUT_Pin) && stateTracker == 1){

			 // Debug message
			 sprintf(msg, "Going to State B.\n\r");
			 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			 // Clear LCD
			 I2C_LCD_Clear(MyI2C_LCD);

			 // Go to State B
			 stateTracker = 2;

		 // Go to State A if it is State B
		 } else if (HAL_GPIO_ReadPin(BUTTON_INPUT_GPIO_Port, BUTTON_INPUT_Pin) && stateTracker == 2){

			 // Debug message
			 sprintf(msg, "Going to State A.\n\r");
			 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			 // Clear LCD
			 I2C_LCD_Clear(MyI2C_LCD);

			 // Stay in State A
			 stateTracker = 1;

		// Button 2 pressed (State C)
		 } else if (!HAL_GPIO_ReadPin(BUTTON_INPUT_2_GPIO_Port, BUTTON_INPUT_2_Pin) && stateTracker == 1){

			 // Debug message
			 sprintf(msg, "Going to State C.\n\r");
			 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			 // Go to State C
			 stateTracker = 3;
		 }
	 }

	 /* State Handler A */
	 void stateHandlerA(void){
		 // LCD Control
		 I2C_LCD_Display(MyI2C_LCD);
		 I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
		 I2C_LCD_WriteString(MyI2C_LCD, "SID: 24429298");
		 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
		 I2C_LCD_WriteString(MyI2C_LCD, "Mechatronics 1");

		 // Get potentiometer value
		 HAL_ADC_Start_IT(&hadc1);	// Start conversion after each ADC cycle
		 getPotValue();

		 // Motor control
		 int servoAngle = myMap(getAdcFromPot(), 60, 4095, 0, 180);
		 motorControl(servoAngle);

		 // No button pushes - Stay in State A
		 stateTracker = 1;
	 }

	 /* State Handler B */
	 void stateHandlerB(void){

		 // When State B is initially called, let alone one of the LEDs (in question) on
		 if (ledOneFlag == 1 && ledTwoFlag == 1){
			 ledOneFlag = 1;
			 ledTwoFlag = 0;
		 }

		 // LCD Control
		 I2C_LCD_Display(MyI2C_LCD);
		 I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
		 I2C_LCD_WriteString(MyI2C_LCD, "ADC: ");
		 sprintf(msg2, "%hu", potValue);
		 I2C_LCD_WriteString(MyI2C_LCD, msg2);
		 I2C_LCD_WriteString(MyI2C_LCD, " State B");
		 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
		 I2C_LCD_WriteString(MyI2C_LCD, "Mechatronics 1");

		 // Get potentiometer value
		 HAL_ADC_Start_IT(&hadc1);	// Start conversion after each ADC cycle
		 getPotValue();

		 // Motor control
		 int servoAngle = myMap(getAdcFromPot(), 60, 4095, 0, 180);
		 motorControl(servoAngle);

		 // LED3 control
		 // servoAngle also mapped to LED3 frequency (PSC)
		 // adc towards 0v = 5hz
		 // adc towards 5v = 1hz

		 // LED1 and LED2 toggle
		 // B1 pressed = Turn off
		 // B1 not pressed = Turn on, blink at 1 Hz (Modify PSC)
		 if (!HAL_GPIO_ReadPin(BUTTON_INPUT_2_GPIO_Port, BUTTON_INPUT_2_Pin)){

			 // If LED1 is on, swap with LED2
			 if (ledOneFlag == 1){
				 ledOneFlag = 0;
				 ledTwoFlag = 1;

			// If LED2 is on, swap with LED1
			 } else if (ledTwoFlag == 1){
				 ledTwoFlag = 0;
				 ledOneFlag = 1;
			 }
		 }

		 // No button pushes - Stay in State B
		 stateTracker = 2;
	 }

	 /* State Handler C */
	 void stateHandlerC(void){

		 // Turn off LCD
		 I2C_LCD_NoDisplay(MyI2C_LCD);

		 // Test function to show State C. State C functionality yet to be added.
		 HAL_Delay(2000);

		 // Reconfigure UART Tx Pin (PA2)
		 // Turn into GPIO (output)
		 // Set pin to toggle output based on TIM with frequency 1 Hz
		 // Cycle for 3 seconds
		 // Turn back into UART (tx)

		 // Return to State A
		 stateTracker = 1;
	 }

	 /* Controller Function for LED1, LED2 and LED3*/
	 // Description: Frequency currently determined by each timer's PSC value.
	 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

		 // TIM2 controls LED1
		 if(htim->Instance == TIM2){
			 if (ledOneFlag == 1){
				 HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
			 }

		 // TIM6 controls LED2
		 } else if(htim->Instance == TIM6){
			 if (ledTwoFlag == 1) {
				 HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
			 }

		 // TIM7 controls LED3
		 } else if(htim->Instance == TIM7){
			 HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);

	     // TIM14 controls UART
		 } else if(htim->Instance == TIM14){
			 if (stateTracker == 1){
				 if (uartFlag == 1){
					 sprintf(msg2, "Autumn2024 MX1 SID: 24429298, ADC Reading: %hu\r\n", potValue);
					 HAL_UART_Transmit(&huart2, (uint8_t*) msg2, strlen(msg2), HAL_MAX_DELAY);
				 }
			 } else if (stateTracker == 2){
				 ; // UART do nothing
			 }
		 }
	 }

	 /* UART Receive Interrupt Handler */
	 // Handles receiving of bytes
	 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

		 sprintf(msg, "Value Received: %s\r\n", msgRx);
		 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		 if (strcmp(msgRx,"j") == 0) {
			 if (stateTracker == 1){

				if (uartFlag == 0){
					sprintf(msg, "Turning ON UART.\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					uartFlag = 1;

				} else if (uartFlag == 1) {
					sprintf(msg, "Turning OFF UART.\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					uartFlag = 0;
				}

			 } else {
				 sprintf(msg, "Cannot disable, not in State A.\r\n");
				 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			 }
		 } else {
			 sprintf(msg, "Incorrect key.\r\n");
			 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		 }
	 }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
