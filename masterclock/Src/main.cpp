/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"
extern "C"
{
	

#include "libjpeg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GUI.h"
#include "mainMenu.h"
#include "guivars.h"
#include "lines.h"
#include "timedate.h"
#include "backup.h"
#include "crc.h"
#include  "adc.h"
#include "touch.h"
#include "sram.h"
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
Variables variables;
uint8_t cnt;
RTC_TimeTypeDef sTime;
RTC_TimeTypeDef sTimePrev;
RTC_DateTypeDef sDate;
RTC_DateTypeDef sDatePrev;
WM_MESSAGE msg;
WM_MESSAGE msgButton;


uint8_t tickSecond = 0;
extern uint16_t backgroundBuffer[18];

GUI_ALLOC_INFO pInfo;
uint8_t doTimeCorrection = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;



TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

osThreadId defaultTaskHandle;
osThreadId GUIHandle;
osThreadId TaskLine0Handle;
osThreadId TaskLine1Handle;
osThreadId TaskLine2Handle;
osThreadId TaskLine3Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_DAC_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void vTaskGUI(void const * argument);
void vTaskLine0(void const * argument);
void vTaskLine1(void const * argument);
void vTaskLine2(void const * argument);
void vTaskLine3(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef * hrtc)
{
	tickSecond = 1;
}
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
	MX_DMA_Init();
	MX_FSMC_Init();
	MX_DAC_Init();
	MX_CRC_Init();
	MX_ADC1_Init();
	MX_RTC_Init();
	MX_I2C1_Init();
	MX_USB_OTG_FS_USB_Init();
	MX_TIM7_Init();
	//MX_SPI2_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	spi2 = new SPI(SPI2, SPI_InitTypeDef{
//		Mode				Direction		DataSize
	SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT,
//		CLKPolarity			CLKPhase		NSS				BaudRatePrescaler
	SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_32,
//		FirstBit			TIMode			CRCCalculation		CRCPolynomial
	 SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_ENABLE, 10});
	touchScreen = new TouchScreen(spi2, GPIOB, 7, GPIOB, 6);
	__TURN_BACKLIGHT_ON;
	variables.calibrated = 0;
	initStructures();
	LinesInit();
	readLinesPolarityFromBKP();
	readSettingsFromBKP();
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
		/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
		/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
		/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of GUI */
	osThreadDef(GUI, vTaskGUI, osPriorityIdle, 0, 600);
	GUIHandle = osThreadCreate(osThread(GUI), NULL);

	/* definition and creation of TaskLine0 */
	osThreadDef(TaskLine0, vTaskLine0, osPriorityNormal, 0, 128);
	TaskLine0Handle = osThreadCreate(osThread(TaskLine0), NULL);

	/* definition and creation of TaskLine1 */
	osThreadDef(TaskLine1, vTaskLine1, osPriorityNormal, 0, 128);
	TaskLine1Handle = osThreadCreate(osThread(TaskLine1), NULL);

	/* definition and creation of TaskLine2 */
	osThreadDef(TaskLine2, vTaskLine2, osPriorityNormal, 0, 128);
	TaskLine2Handle = osThreadCreate(osThread(TaskLine2), NULL);

	/* definition and creation of TaskLine3 */
	osThreadDef(TaskLine3, vTaskLine3, osPriorityIdle, 0, 128);
	TaskLine3Handle = osThreadCreate(osThread(TaskLine3), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
		/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
		/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
 

	/* Start scheduler */
	osKernelStart();
  
	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure the analog watchdog 
	*/
	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
	AnalogWDGConfig.HighThreshold = 3500;
	AnalogWDGConfig.LowThreshold = 300;
	AnalogWDGConfig.ITMode = ENABLE;
	if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	*/
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	*/
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	*/
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 4;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/**DAC Initialization 
	*/
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}
	/**DAC channel OUT1 config 
	*/
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/**Initialize RTC Only 
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */
	if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0)
	{
		sTime.Hours = 0;
		sTime.Minutes = 0;
		sTime.Seconds = 0;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 1;
		sDate.Year = 0;
	
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
	}
	/* USER CODE END Check_RTC_BKUP */

	/**Initialize RTC and set the Time and Date 
	*/
		
		/**Enable the WakeUp 
		*/
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
		Error_Handler();
	}
	/**Enable the reference Clock input 
	*/
	if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 100;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 62999;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LINE0_NEG_OUTPUT_Pin | LINE1_NEG_OUTPUT_Pin | LINE2_NEG_OUTPUT_Pin | LINE3_NEG_OUTPUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, LINE0_POS_OUTPUT_Pin | LINE1_POS_OUTPUT_Pin | LINE2_POS_OUTPUT_Pin | LINE3_POS_OUTPUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
		RELAY1_Pin|RELAY2_Pin|RELAY3_Pin|LCD_RESET_Pin 
	                        |BACKLIGHT_CONTROL_Pin,
		GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RELAY0_GPIO_Port, RELAY0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LINE0_NEG_OUTPUT_Pin LINE1_NEG_OUTPUT_Pin LINE2_NEG_OUTPUT_Pin LINE3_NEG_OUTPUT_Pin */
	GPIO_InitStruct.Pin = LINE0_NEG_OUTPUT_Pin | LINE1_NEG_OUTPUT_Pin | LINE2_NEG_OUTPUT_Pin | LINE3_NEG_OUTPUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LINE0_POS_OUTPUT_Pin LINE1_POS_OUTPUT_Pin LINE2_POS_OUTPUT_Pin LINE3_POS_OUTPUT_Pin */
	GPIO_InitStruct.Pin = LINE0_POS_OUTPUT_Pin | LINE1_POS_OUTPUT_Pin | LINE2_POS_OUTPUT_Pin | LINE3_POS_OUTPUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : RELAY1_Pin RELAY2_Pin RELAY3_Pin LCD_RESET_Pin */
	GPIO_InitStruct.Pin = RELAY1_Pin | RELAY2_Pin | RELAY3_Pin | LCD_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : RELAY0_Pin */
	GPIO_InitStruct.Pin = RELAY0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RELAY0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA10 PA11 PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BACKLIGHT_CONTROL_Pin */
	GPIO_InitStruct.Pin = BACKLIGHT_CONTROL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BACKLIGHT_CONTROL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NOT_PEN_Pin */
	GPIO_InitStruct.Pin = NOT_PEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NOT_PEN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_CS_Pin */
	GPIO_InitStruct.Pin = TOUCH_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
	FSMC_NORSRAM_TimingTypeDef Timing;

	/** Perform the SRAM1 memory initialization sequence
	*/
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 2;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 5;
	Timing.BusTurnAroundDuration = 0;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
	{
		Error_Handler();
	}

	/** Perform the SRAM2 memory initialization sequence
	*/
	hsram2.Instance = FSMC_NORSRAM_DEVICE;
	hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram2.Init */
	hsram2.Init.NSBank = FSMC_NORSRAM_BANK3;
	hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram2.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 0;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 8;
	Timing.BusTurnAroundDuration = 0;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* init code for LIBJPEG */
	//MX_LIBJPEG_Init();

	/* USER CODE BEGIN 5 */
	GUI_PID_STATE touchState;
	const Touch::Coords *readedCoords;

	/* Infinite loop */
	for (;;)
	{
		osDelay(50);

		readedCoords=touchScreen->getXY(Touch::Calibrated::CALIBRATED);
		if (touchScreen->isCoordsOk())
		{
			touchState.Pressed = true;
			touchState.x = readedCoords->X;
			touchState.y = readedCoords->Y;
			GUI_TOUCH_StoreStateEx(&touchState);
		}
		refreshCurrentMeter(masterClock.currentSense);
		if (masterClock.longPressCNT->it)
		{
			longPressControl();
		}
	}
	/* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_vTaskGUI */
/**
* @brief Function implementing the GUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskGUI */
void vTaskGUI(void const * argument)
{
	/* USER CODE BEGIN vTaskGUI */
	uint16_t i = 0;

	GUI_Init();
	Calibrate(&variables);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	CreateMainMenu();
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	masterClock.line[0].TimeZone = masterClock.daylightSaving->timeZone;
	pollLinesOutput(10);
	/* Infinite loop */
	for (; ;)
	{

		GUI_Delay(250);
		GUI_ALLOC_GetMemInfo(&pInfo);

		if (tickSecond) //см. callback  HAL_RTCEx_RTCEventCallback
			{
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				//калибровка
				if(sTime.Hours == 0 && sTime.Minutes == 0 && sTime.Seconds == 0)
				{
					timeCalibr.isCalibrated = false;
					masterClock.daylightSaving->needToShift = true;
				}
				//коррекция происходит в 01:02:00
				if(doTimeCorrection || (sTime.Hours == 1 && sTime.Minutes == 2 && sTime.Seconds == 30))
				{

					if (masterClock.timeCalibration->seconds != 0 && masterClock.timeCalibration->days != 0 && timeCalibr.isCalibrated == false) //если калибровка включена
						{
							masterClock.timeCalibration->daysPassed++;
							if (masterClock.timeCalibration->daysPassed == masterClock.timeCalibration->days) //если настал день калибровки
								{
									if (masterClock.timeCalibration->seconds > 0) //если добавить секунды
										{
											sTime.Seconds += masterClock.timeCalibration->seconds;                                         //прибавили секунды
											if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
											{
												Error_Handler();
											}
										}
									if (masterClock.timeCalibration->seconds < 0) //если убавить секунды
										{
											sTime.Seconds += masterClock.timeCalibration->seconds;                                          //прибавили секунды
											if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
											{
												Error_Handler();
											}
										}
									masterClock.timeCalibration->daysPassed = 0;                                         // 1 => 0
									timeCalibr.isCalibrated = true;
								}
						}
					if (doTimeCorrection || (masterClock.daylightSaving->needToShift&& masterClock.daylightSaving->enableDLS&&isDaylightSavingTimeEU(sDate.Date, sDate.Month, sDate.WeekDay)))
					{
						sTime.Hours += masterClock.daylightSaving->timeShift;
						pollLinesOutput(10);
						masterClock.daylightSaving->needToShift = false;
						if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
						{
							Error_Handler();
						}
					}
					doTimeCorrection = false;

				}
				if (sTimePrev.Seconds == 59) //каждую минуту
					{
						for (i = 0; i < LINES_AMOUNT; ++i)
						{
							if (masterClock.line[i].Pulses >= 0)
							{
								xSemaphoreGive(masterClock.line[i].xSemaphore);
							}
							else
							{
								masterClock.line[i].Pulses++;
							}
						}
					}
				if (sTimePrev.Hours == 23 && sTime.Hours == 0) //сменился день
					{
						saveDateToBKP();
						switch (masterClock.guiVars->menuState) {
						case MENU_STATE_MAIN:
							sendMsg(masterClock.handles->hMainMenu, WM_DATE_UPDATE);
							break;
						case MENU_STATE_TIMEDATESETUP:
							sendMsg(masterClock.handles->hTimeDateSetupMenu, WM_DATE_UPDATE);
							break;
						}
					}
				switch (masterClock.guiVars->menuState) {
				case MENU_STATE_MAIN:
					sendMsg(masterClock.handles->hMainMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hMainMenu);
					break;
				case MENU_STATE_TIMESETUP:
					sendMsg(masterClock.handles->hTimeSetupMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hTimeSetupMenu);
					break;
				case MENU_STATE_LINE1SETUP:
					sendMsg(masterClock.handles->hLineSetupMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupMenu);
					break;
				case MENU_STATE_LINE2SETUP:
					sendMsg(masterClock.handles->hLineSetupMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupMenu);
					break;
				case MENU_STATE_LINE3SETUP:
					sendMsg(masterClock.handles->hLineSetupMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupMenu);
					break;
				case MENU_STATE_LINE4SETUP:
					sendMsg(masterClock.handles->hLineSetupMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupMenu);
					break;
				case MENU_STATE_LINE1SETUP_PULSE:
					sendMsg(masterClock.handles->hLineSetupPulseMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupPulseMenu);
					break;
				case MENU_STATE_LINE2SETUP_PULSE:
					sendMsg(masterClock.handles->hLineSetupPulseMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupPulseMenu);
					break;
				case MENU_STATE_LINE3SETUP_PULSE:
					sendMsg(masterClock.handles->hLineSetupPulseMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupPulseMenu);
					break;
				case MENU_STATE_LINE4SETUP_PULSE:
					sendMsg(masterClock.handles->hLineSetupPulseMenu, WM_SEC_UPDATE);
					menuLocker(&masterClock.handles->hLineSetupPulseMenu);
					break;
				case MENU_STATE_PASSWORD:
					sendMsg(masterClock.handles->hPasswordMenu, WM_SEC_UPDATE);
					break;
				case MENU_STATE_SETTINGS:
					menuLocker(&masterClock.handles->hLineSetupPulseMenu);
					break;

				}
				tickSecond = 0;
				sTimePrev = sTime;
			}

	}
	/* USER CODE END vTaskGUI */
}

/* USER CODE BEGIN Header_vTaskLine0 */
/**
* @brief Function implementing the TaskLine0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLine0 */
void vTaskLine0(void const * argument)
{
	/* USER CODE BEGIN vTaskLine0 */
		/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(masterClock.line[0].xSemaphore, portMAX_DELAY);
		lineSendSignal(0);
		saveLineToBKP(0);
	}
	/* USER CODE END vTaskLine0 */
}

/* USER CODE BEGIN Header_vTaskLine1 */
/**
* @brief Function implementing the TaskLine1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLine1 */
void vTaskLine1(void const * argument)
{
	/* USER CODE BEGIN vTaskLine1 */
		/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(masterClock.line[1].xSemaphore, portMAX_DELAY);
		lineSendSignal(1);
		saveLineToBKP(1);
	}
	/* USER CODE END vTaskLine1 */
}

/* USER CODE BEGIN Header_vTaskLine2 */
/**
* @brief Function implementing the TaskLine2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLine2 */
void vTaskLine2(void const * argument)
{
	/* USER CODE BEGIN vTaskLine2 */
		/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(masterClock.line[2].xSemaphore, portMAX_DELAY);
		lineSendSignal(2);
		saveLineToBKP(2);
	}
	/* USER CODE END vTaskLine2 */
}

/* USER CODE BEGIN Header_vTaskLine3 */
/**
* @brief Function implementing the TaskLine3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLine3 */
void vTaskLine3(void const * argument)
{
	/* USER CODE BEGIN vTaskLine3 */
		/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(masterClock.line[3].xSemaphore, portMAX_DELAY);
		lineSendSignal(3);
		saveLineToBKP(3);
	}
	/* USER CODE END vTaskLine3 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */

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
		   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
