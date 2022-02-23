/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <DS18B20.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SETCODE "SetTime"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__IO uint32_t T1 = 2000;
uint16_t u16_ADCVal;
float f_Humi;
uint8_t Rx_Buffer[20] = "";
uint8_t Rx_data;
uint8_t _rxIndex;
uint8_t Rx_Flag = 0;
uint8_t CheckSet = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for SetTimeTask */
osThreadId_t SetTimeTaskHandle;
const osThreadAttr_t SetTimeTask_attributes = {
  .name = "SetTimeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TempTask */
osThreadId_t TempTaskHandle;
const osThreadAttr_t TempTask_attributes = {
  .name = "TempTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for HumiTask */
osThreadId_t HumiTaskHandle;
const osThreadAttr_t HumiTask_attributes = {
  .name = "HumiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LightTask */
osThreadId_t LightTaskHandle;
const osThreadAttr_t LightTask_attributes = {
  .name = "LightTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for BinarySem */
osSemaphoreId_t BinarySemHandle;
const osSemaphoreAttr_t BinarySem_attributes = {
  .name = "BinarySem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
void StartSetTimeTask(void *argument);
void StartTempTask(void *argument);
void StartHumiTask(void *argument);
void StartLightTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct __FILE
{
	int handle;
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int ferror(FILE *f)
{
	/* Your implementation of ferror(). */
	return 0;
}
void do_SetTimeTask (void)
{
	uint8_t data[]="\nSetTimeTask is running .. ";
	HAL_UART_Transmit(&huart1,data,sizeof(data)-1, 100);
}

void do_TempTask (void)
{
	uint8_t data[]="\nTempTask is running .. ";
	HAL_UART_Transmit(&huart1,data,sizeof(data)-1, 100);
}

void do_HumiTask (void)
{
	uint8_t data[]="\nHumiTask is running .. ";
	HAL_UART_Transmit(&huart1,data,sizeof(data)-1, 100); 
}
void do_LightTask (void)
{
	uint8_t data[]="\nLightTask is running .. ";
	HAL_UART_Transmit(&huart1,data,sizeof(data)-1, 100);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart -> Instance == USART1)
	{
		if(Rx_data != 13)
		{
			Rx_Buffer[_rxIndex++] = Rx_data;
		}
		else if(Rx_data == 13)
		{
			_rxIndex = 0;
			Rx_Flag = 1;
		}
		HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
	}
}
void CheckSetCode(uint8_t *dtpr)
{
	uint8_t sum = 0;						//Tong kiem tra
	for(int i = 0 ; i < 7; i ++)
	{
		sum += dtpr[i] - (uint8_t)SETCODE[i];//Cong hieu tung ky tu cua 2 mang
	}
	if(sum == 0)			//Tong hieu bang 0 => hai mang tuong tu
		CheckSet = 1;
	else if (sum != 0)
		CheckSet = 0;		//Tong hieu khac 0 => Hai mang khac nhau
}

void getTime(uint8_t *dtpr) //SetTime 2000 <CR>
{
	uint32_t sum = 0;				//Gia tri SetTime
	uint8_t Time_array[12];	//Mang luu tru ky tu gia tri SetTime
	uint8_t counter = 0;
	for(int i = 8; i < 20; i++)					//Kiem tra cac ky tu la so
	{
		if(dtpr[i] <= 57 && dtpr[i] >= 48)
		{
			Time_array[i-8] = dtpr[i] - 48;	//Doi ve gia tri int
			counter++;											//Neu la ky tu so thi tang counter
		}
		else if( dtpr[i] >57 || dtpr[i] < 48)
		{
			break;
		}		
	}	
	for(int i = 0; i < counter; i++)
	{
		sum += Time_array[i]*pow(10,(double)(counter - i - 1));	//Tinh gia tri SetTime
	}
	T1 = sum;		//Gan gia tri cho chu ky T1
	}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc -> Instance == ADC1)
	{
		u16_ADCVal = HAL_ADC_GetValue(&hadc1);
		f_Humi = (100.0 - (float)(u16_ADCVal)/4095*100);
	}
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,&Rx_data,1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem */
  BinarySemHandle = osSemaphoreNew(1, 1, &BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SetTimeTask */
  SetTimeTaskHandle = osThreadNew(StartSetTimeTask, NULL, &SetTimeTask_attributes);

  /* creation of TempTask */
  TempTaskHandle = osThreadNew(StartTempTask, NULL, &TempTask_attributes);

  /* creation of HumiTask */
  HumiTaskHandle = osThreadNew(StartHumiTask, NULL, &HumiTask_attributes);

  /* creation of LightTask */
  LightTaskHandle = osThreadNew(StartLightTask, NULL, &LightTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSetTimeTask */
/**
  * @brief  Function implementing the SetTimeTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSetTimeTask */
void StartSetTimeTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	__IO uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {	
		osSemaphoreAcquire(BinarySemHandle, osWaitForever);
		if(Rx_Flag == 1)
		{
			printf("\n%s @%d: Enter!",SetTimeTask_attributes.name,osKernelGetTickCount());
		
			do_SetTimeTask();
			printf("\nCommand: %s",Rx_Buffer);
			
			CheckSetCode(Rx_Buffer);
			if(CheckSet == 1)
			{
				getTime(Rx_Buffer);
				printf("\nNow period time : %d",T1);
				printf("\nPress Button to Confirm Change!");				

				while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin) == GPIO_PIN_RESET);
				printf("\nChange is confirmed!");
			}
			else if (CheckSet == 0)
			{
				printf("\nWrong Command!");
				printf("\nNow period time : %d",T1);
			}
			Rx_Flag = 0;
			tick+= 1000;
			printf("\n%s @%d: exit!\n",SetTimeTask_attributes.name,osKernelGetTickCount());
		}
		else if( Rx_Flag == 0)
		{
			tick += 1000;
		}
		osSemaphoreRelease(BinarySemHandle);
		osDelayUntil(tick);
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
void StartTempTask(void *argument)
{
  /* USER CODE BEGIN StartTempTask */
	//DS18B20 Init
	DS18B20_Name DS1;
	DS18B20_Init(&DS1, &htim3, DS18B20_GPIO_Port, DS18B20_Pin);
	
	//Variable
	__IO uint32_t tick = osKernelGetTickCount();
	float f_Temp;
  /* Infinite loop */
  for(;;)
  {	
		osSemaphoreAcquire(BinarySemHandle, osWaitForever);
		printf("\n%s @%d: Enter!",TempTask_attributes.name,osKernelGetTickCount());

		do_TempTask();

    f_Temp = DS18B20_ReadTemp(&DS1);
		printf("\nTemp: %2.2f",f_Temp);
	
		printf("\n%s @%d: Exit!\n",TempTask_attributes.name,osKernelGetTickCount());
		osSemaphoreRelease(BinarySemHandle);
		tick += T1;
		osDelayUntil(tick);
		
  }
	
  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartHumiTask */
/**
* @brief Function implementing the HumiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHumiTask */
void StartHumiTask(void *argument)
{
  /* USER CODE BEGIN StartHumiTask */
	__IO uint32_t tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {	
		osSemaphoreAcquire(BinarySemHandle, osWaitForever);
		printf("\n%s @%d: Enter!",HumiTask_attributes.name,osKernelGetTickCount());
		tick += T1;	
		do_HumiTask();
		
		HAL_ADC_Start_IT(&hadc1);
		HAL_Delay(50);
		printf("\nHumidity: %2.2f%%",f_Humi);

		printf("\n%s @%d: Exit!\n",HumiTask_attributes.name,osKernelGetTickCount());
		
		osSemaphoreRelease(BinarySemHandle);	
			
		osDelayUntil(tick);
  }
  /* USER CODE END StartHumiTask */
}

/* USER CODE BEGIN Header_StartLightTask */
/**
* @brief Function implementing the LightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightTask */
void StartLightTask(void *argument)
{
  /* USER CODE BEGIN StartLightTask */
	__IO uint32_t tick = osKernelGetTickCount();
	float f_Light = 65000;
  /* Infinite loop */
  for(;;)
  {	
		osSemaphoreAcquire(BinarySemHandle, osWaitForever);
    printf("\n%s @%d: Enter!",LightTask_attributes.name,osKernelGetTickCount());	
		do_LightTask();
		
		printf("\nLight: %2.2f",f_Light);
		
		printf("\n%s @%d: Exit!\n",LightTask_attributes.name,osKernelGetTickCount());
		osSemaphoreRelease(BinarySemHandle);
		
		tick += T1;
		osDelayUntil(tick);
  }
  /* USER CODE END StartLightTask */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
