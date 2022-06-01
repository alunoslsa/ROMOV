/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include <stm32l4xx_it.h>
/*Mutex includes*/
#include "semphr.h"
#include "queue.h"

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
UART_HandleTypeDef huart2;

/* Definitions for SendOdom */
osThreadId_t SendOdomHandle;
const osThreadAttr_t SendOdom_attributes = {
  .name = "SendOdom",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SendMessage */
osThreadId_t SendMessageHandle;
const osThreadAttr_t SendMessage_attributes = {
  .name = "SendMessage",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xQueueUSART */
osMessageQueueId_t xQueueUSARTHandle;
const osMessageQueueAttr_t xQueueUSART_attributes = {
  .name = "xQueueUSART"
};
/* Definitions for xSemaphoreBiEncoder */
osSemaphoreId_t xSemaphoreBiEncoderHandle;
const osSemaphoreAttr_t xSemaphoreBiEncoder_attributes = {
  .name = "xSemaphoreBiEncoder"
};
/* Definitions for xSemaphoreBiSync */
osSemaphoreId_t xSemaphoreBiSyncHandle;
const osSemaphoreAttr_t xSemaphoreBiSync_attributes = {
  .name = "xSemaphoreBiSync"
};
/* USER CODE BEGIN PV */
int32_t Wheel_R, Wheel_L;
int QEM[16]={0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
uint8_t Wheel_R_Old=0, Wheel_R_New=0, Wheel_L_Old=0, Wheel_L_New=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void prvSendOdom(void *argument);
void prvSendMessage(void *argument);

/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xSemaphoreBiEncoder */
  xSemaphoreBiEncoderHandle = osSemaphoreNew(1, 1, &xSemaphoreBiEncoder_attributes);

  /* creation of xSemaphoreBiSync */
  xSemaphoreBiSyncHandle = osSemaphoreNew(1, 1, &xSemaphoreBiSync_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueueUSART */
  xQueueUSARTHandle = osMessageQueueNew (3, 20, &xQueueUSART_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SendOdom */
  SendOdomHandle = osThreadNew(prvSendOdom, NULL, &SendOdom_attributes);

  /* creation of SendMessage */
  SendMessageHandle = osThreadNew(prvSendMessage, NULL, &SendMessage_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin==GPIO_PIN_9){
		static BaseType_t pxHigherPriorityTaskWoken;

		xSemaphoreGiveFromISR( xSemaphoreBiSyncHandle, &pxHigherPriorityTaskWoken );

		if( pxHigherPriorityTaskWoken == pdTRUE )
			taskYIELD(); /* forces the context change */

	}else if ((GPIO_Pin==GPIO_PIN_0)||(GPIO_Pin==GPIO_PIN_1)){
		static BaseType_t pxHigherPriorityTaskWoken;

		Wheel_L_Old = Wheel_L_New;

		xSemaphoreTakeFromISR(xSemaphoreBiEncoderHandle, &pxHigherPriorityTaskWoken);

		Wheel_L_New = 2*HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) + HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		Wheel_L += QEM[Wheel_L_Old*4 + Wheel_L_New];
		xSemaphoreGiveFromISR( xSemaphoreBiEncoderHandle, &pxHigherPriorityTaskWoken );

		if( pxHigherPriorityTaskWoken == pdTRUE )
			taskYIELD(); /* forces the context change */
	}
	else if ((GPIO_Pin==GPIO_PIN_2)||(GPIO_Pin==GPIO_PIN_3)){
		static BaseType_t pxHigherPriorityTaskWoken;

		Wheel_R_Old = Wheel_R_New;

		xSemaphoreTakeFromISR(xSemaphoreBiEncoderHandle, &pxHigherPriorityTaskWoken);

		Wheel_R_New = 2*HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) + HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		Wheel_R += QEM[Wheel_R_Old*4 + Wheel_R_New];

		xSemaphoreGiveFromISR( xSemaphoreBiEncoderHandle, &pxHigherPriorityTaskWoken );

		if( pxHigherPriorityTaskWoken == pdTRUE )
			taskYIELD(); /* forces the context change */
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_prvSendOdom */
/**
* @brief Function implementing the SendOdom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvSendOdom */
void prvSendOdom(void *argument)
{
  /* USER CODE BEGIN 5 */
	char message[20];
	int32_t Wheel_R_cpy, Wheel_L_cpy;
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);

	  xSemaphoreTake(xSemaphoreBiEncoderHandle, ( TickType_t) portMAX_DELAY);
	  Wheel_R_cpy = Wheel_R;
	  Wheel_L_cpy = Wheel_L;

	  Wheel_L = 0;
	  Wheel_R = 0;
      xSemaphoreGive(xSemaphoreBiEncoderHandle);

	  //Create message
	  sprintf(message, "%lu %li %li\n", xTaskGetTickCount(), Wheel_L_cpy, Wheel_R_cpy);

	  xQueueSendToBack( xQueueUSARTHandle, &message, ( TickType_t ) portMAX_DELAY );

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_prvSendMessage */
/**
* @brief Function implementing the SendMessage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvSendMessage */
void prvSendMessage(void *argument)
{
  /* USER CODE BEGIN prvSendMessage */
	char message[20];
  /* Infinite loop */
  for(;;)
  {
	  /* Block until receive message. */
	  xQueueReceive( xQueueUSARTHandle, &message, ( TickType_t ) portMAX_DELAY );
	  HAL_UART_Transmit(&huart2, &message, strlen( message ), 20);

  }
  /* USER CODE END prvSendMessage */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

