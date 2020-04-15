/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId LevelTaskHandle;
osThreadId TempTaskHandle;
osThreadId pHTaskHandle;
osThreadId ComTaskHandle;

osMailQId SCADAHandle;
osMutexId MutexADCHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
void StartLevelTask(void const * argument);
void StartTempTask(void const * argument);
void StartpHTask(void const * argument);
void StartComTask(void const * argument);

/* USER CODE BEGIN PFP */
//uint8_t bufferZaSlanjeLevel [6];
//uint8_t bufferZaSlanjeTemp [6];
//uint8_t bufferZaSlanjePH [6];

uint16_t adcValue[3];
osStatus status = osOK;

static float LevelMax;
static float phMax;
static float tempMax;

static float phTrenutno;
static float tempTrenutno;
static float LevelTrenutno;

uint32_t Ttimestamp;
uint32_t Ptimestamp;
uint32_t Ltimestamp;

unsigned char Ulazni_buffer[8];
unsigned char Izlazni_buffer[25];

struct dolaznaPoruka {
	char tip;
	char vrijednost[7];
};

struct odlaznaPoruka{
	char tip;
	char refVrijednost[7];
	char mjVrijednost[7];
	char timestamp[10];
};

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
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexADC */
  osMutexDef(MutexADC);
  MutexADCHandle = osMutexCreate(osMutex(MutexADC));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of SCADA */
  //osMailQDef(SCADA, 16, Data);
	//SCADAHandle = osMailCreate(osMailQ(SCADA), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LevelTask */
  osThreadDef(LevelTask, StartLevelTask, osPriorityAboveNormal, 0, 128);
  LevelTaskHandle = osThreadCreate(osThread(LevelTask), NULL);

  /* definition and creation of TempTask */
  osThreadDef(TempTask, StartTempTask, osPriorityHigh, 0, 128);
  TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);

  /* definition and creation of pHTask */
  osThreadDef(pHTask, StartpHTask, osPriorityRealtime, 0, 128);
  pHTaskHandle = osThreadCreate(osThread(pHTask), NULL);

  /* definition and creation of ComTask */
  osThreadDef(ComTask, StartComTask, osPriorityLow, 0, 128);
  ComTaskHandle = osThreadCreate(osThread(ComTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adcValue, 3);
  
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLevelTask */
/**
  * @brief  Function implementing the LevelTask thread.
  * @param  argument: Not used 
  * @retval None
  */

void pwm(uint16_t val){
		
		TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = val;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  
}

/* USER CODE END Header_StartLevelTask */
void StartLevelTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	
	ADC_ChannelConfTypeDef sConfig = {0};
	static float b0 = 0.2;
	static float b1 = 0.4;
	static float b2 = 0.6;
	static float ek_1 = 0;
	static float ek_2 = 0;
	static float uk_1 = 0;
	uint16_t izmjereniLevel; 
 
	for(;;)
  {
		int start = osKernelSysTick();
		if(status == osOK)
			{
				status = osMutexWait(MutexADCHandle,2000);
				sConfig.Channel = ADC_CHANNEL_6;
				sConfig.Rank = 1;
				
				if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) 
					Error_Handler();
				
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2,10UL);
				adcValue[0]=HAL_ADC_GetValue(&hadc2);  
				HAL_ADC_Stop(&hadc2);
				status = osMutexRelease(MutexADCHandle);
			}
		
		/*
		for(int i = 0; i < 4; ++i)
		{
			bufferZaSlanjeLevel[4-i] = (adcValue[0] % 10)+48;
			adcValue[0]/=10;
		}
		*/
			
			izmjereniLevel = adcValue[0];
			float ek = LevelMax - izmjereniLevel;
			float uk = uk_1 + b0 * ek + b1 * ek_1 + b2 * ek_2;
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, uk);
			ek_2 = ek_1;
			ek_1 = ek;
			uk_1 = uk;
		
		/*
		bufferZaSlanjeLevel[5] = '\n';
		bufferZaSlanjeLevel[0] = 'L';
		HAL_UART_Transmit(&huart1,bufferZaSlanjeLevel,6,100);
		*/
		
			int end = osKernelSysTick();
			Ltimestamp = end-start;
			osDelay(100 - Ltimestamp);
		}
		/* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTempTask */
/**
* @brief Function implementing the TempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempTask */
void StartTempTask(void const * argument)
{
  /* USER CODE BEGIN StartTempTask */
  /* Infinite loop */
	
	ADC_ChannelConfTypeDef sConfig = {0};
	static float b0 = 0.4;
	static float b1 = 0.3;
	static float b2 = 0.5;
	static float ek_1 = 0;
	static float ek_2 = 0;
	static float uk_1 = 0; 
	uint16_t temperaturaIzmjerena; 
 
	for(;;)
  {
		int start = osKernelSysTick();
		if(status == osOK)
			{
				status = osMutexWait(MutexADCHandle,2000);
				sConfig.Channel = ADC_CHANNEL_7;
				sConfig.Rank = 1;
				
				if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
					Error_Handler();
		
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2,10UL);
				adcValue[1]=HAL_ADC_GetValue(&hadc2);  
				status = osMutexRelease(MutexADCHandle);
			}
		
		/*		
		for(int i = 0; i < 4; ++i)
		{
			bufferZaSlanjeTemp[4-i] = (adcValue[1] % 10)+48;
			adcValue[1]/=10;
		}
		*/
		
		temperaturaIzmjerena = adcValue[1];
		float ek = tempMax - temperaturaIzmjerena;
		float uk = uk_1 + b0 * ek + b1 * ek_1 + b2 * ek_2;
		
		pwm(uk);
		ek_2 = ek_1;
		ek_1 = ek;
		uk_1 = uk;
		
		/*
		bufferZaSlanjeTemp[5] = '\n';
		bufferZaSlanjeTemp[0] = 'T';
		HAL_UART_Transmit(&huart1,bufferZaSlanjeTemp,6,100);
		*/
	
		int end = osKernelSysTick();
		Ttimestamp = end - start;
		osDelay(70 - Ttimestamp);
  }
  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartpHTask */
/**
* @brief Function implementing the pHTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartpHTask */
void StartpHTask(void const * argument)
{
  /* USER CODE BEGIN StartpHTask */
  /* Infinite loop */
  
	static float b0 = 0.2;
	static float b1 = 0.4;
	static float b2 = 0.6;
	static float ek_1 = 0;// e(k-1)
	static float ek_2 = 0;// e(k-2)
	static float uk_1 = 0;// u(k-1)
	uint16_t phIzmjerena; 

	ADC_ChannelConfTypeDef sConfig = {0};
	for(;;)
  {
		int start = osKernelSysTick();
		if(status == osOK)
			{
				status = osMutexWait(MutexADCHandle,2000);
				sConfig.Channel = ADC_CHANNEL_8;
				sConfig.Rank = 1;
			
				if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
					Error_Handler();
				
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2,10UL);
				adcValue[2]=HAL_ADC_GetValue(&hadc2);  
				status = osMutexRelease(MutexADCHandle);
			}
		
			phIzmjerena = adcValue[2];
			float ek = phMax - phIzmjerena;
			float uk = uk_1 + b0 * ek + b1 * ek_1 + b2 * ek_2;
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, uk);
			ek_2 = ek_1;
			ek_1 = ek;
			uk_1 = uk;
		
		/*	
		for(int i = 0; i < 4; i++){
			bufferZaSlanjePH[4-i] = (adcValue[2] % 10)+48;
			adcValue[2]/=10;
		}
		
		bufferZaSlanjePH[5] = '\n';
		bufferZaSlanjePH[0] = 'P';
		HAL_UART_Transmit(&huart1,bufferZaSlanjePH,6,100);
		*/
		
		int end = osKernelSysTick();
	  Ptimestamp = end-start;
		osDelay(50- Ptimestamp);
  }
  /* USER CODE END StartpHTask */
}

/* USER CODE BEGIN Header_StartComTask */
/**
* @brief Function implementing the ComTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComTask */

void print (int num)
{
	struct odlaznaPoruka Izlaz;
	Izlaz.tip = num;
	
	float Max = 0,Trenutno = 0;
	uint32_t timestamp = 0;
	if(num == 1)
	{
			Max = phMax;
			Trenutno = phTrenutno;
			timestamp = Ptimestamp;
	}
	else if(num == 2)
	{
			Max = tempMax;
			Trenutno = tempTrenutno;
			timestamp = Ttimestamp;
	}
	else if(num == 3)
	{
			Max = LevelMax;
			Trenutno = LevelTrenutno;
			timestamp = Ltimestamp;
	}
	
	sprintf(Izlaz.refVrijednost, "%3.3f", Max); 
	sprintf(Izlaz.mjVrijednost, "%3.3f", Trenutno); 
	sprintf(Izlaz.timestamp, "%10d", timestamp);
	memcpy(Izlazni_buffer, &Izlaz, sizeof(Izlaz));
	HAL_UART_Transmit(&huart1, Izlazni_buffer, 25, 1000);
	
}

void StartComTask(void const * argument)
{
  /* USER CODE BEGIN StartComTask */
  /* Infinite loop */
	for(;;)
  {
		HAL_UART_Receive(&huart1, Ulazni_buffer, 8, 10000);
		struct dolaznaPoruka *Ulaz = (struct dolaznaPoruka *) Ulazni_buffer;
		if(Ulaz->tip == 1)
			{
				phMax = strtof(Ulaz->vrijednost, NULL);
			}	
				else if(Ulaz->tip == 2)
				{
					tempMax = strtof(Ulaz->vrijednost, NULL);
				}
					else if(Ulaz->tip == 3)
						{
							LevelMax = strtof(Ulaz->vrijednost, NULL);
						}
							else
								{
									print(1);
									print(2);
									print(3);
								}
  }
  /* USER CODE END StartComTask */
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
