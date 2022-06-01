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
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/*Mutex includes*/
#include "semphr.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Send encoder ticks every 100 Hz. */
static void prvSendOdom( void *pvParameters );

/* Send Lidar Sync signal. */
static void prvSendLidarSync( void *pvParameters );

/* Send message to USART 2. */
static void prvSendMessage( void *pvParameters );

/* Send message to USART 2. */
static void prvToggle_LED( void *pvParameters );

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Task 1 handle variable. */
TaskHandle_t HandleTask1;

/* Task 2 handle variable. */
TaskHandle_t HandleTask2;

/* Task 3 handle variable. */
TaskHandle_t HandleTask3;

/* Semaphore handle variable */
SemaphoreHandle_t xSemaphoreBiEncoder, xSemaphoreBiSync;

/* Data Queue handle variable */
QueueHandle_t xQueueUSART;

int32_t Wheel_R, Wheel_L;
int QEM[16]={0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; //Quadrature Encoder Matrix
uint8_t Wheel_R_Old=0, Wheel_R_New=0, Wheel_L_Old=0, Wheel_L_New=0;
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
  /* Create Binary Semaphore for Encoder Variables */
      xSemaphoreBiEncoder = xSemaphoreCreateBinary();

      /* Create Binary Semaphore for Encoder Variables */
      xSemaphoreBiSync = xSemaphoreCreateBinary();

      xQueueUSART = xQueueCreate( 10, sizeof( char [20] ) );
      if( xQueueUSART == 0 ) {
      	return 0;
      }

  	/* Create the tasks */
   	xTaskCreate( prvSendOdom, "SendOdom", configMINIMAL_STACK_SIZE+400, NULL, 1, &HandleTask1 );

   	xTaskCreate( prvSendLidarSync, "SendLidarSync", configMINIMAL_STACK_SIZE+400, NULL, 1, &HandleTask2 );

   	xTaskCreate( prvSendMessage, "SendMessage", configMINIMAL_STACK_SIZE+400, NULL, 1, &HandleTask3 );

   	xTaskCreate( prvToggle_LED, "toggle_LED", configMINIMAL_STACK_SIZE, NULL, 1, &HandleTask3 );

  	/* Start the scheduler. */
  	vTaskStartScheduler();

  	/* Will only get here if there was not enough heap space to create the idle task. */
  	return 0;
  /* USER CODE END 2 */

  /* Init scheduler */


  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
static void prvToggle_LED( void *pvParameters ){
	for(;;){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
}

static void prvSendOdom( void *pvParameters )
{
	char message[20];
    int32_t Wheel_R_cpy, Wheel_L_cpy;



    for( ;; )
	{
		/* Block 10 milliseconds. */
    	osDelay(10);

		xSemaphoreTake(xSemaphoreBiEncoder, ( TickType_t) portMAX_DELAY);

		Wheel_R_cpy = Wheel_R;
		Wheel_L_cpy = Wheel_L;

		Wheel_L = 0;
		Wheel_R = 0;

		xSemaphoreGive(xSemaphoreBiEncoder);

		//Create message
		sprintf(message, "%lu %li %li\n", xTaskGetTickCount(), Wheel_L_cpy, Wheel_R_cpy);

		xQueueSendToBack( xQueueUSART, &message, ( TickType_t ) portMAX_DELAY );

	}
}
/*-----------------------------------------------------------*/


static void prvSendLidarSync( void *pvParameters )
{
	/* Stops execution of the code below on first run. */
	xSemaphoreTake(xSemaphoreBiSync, ( TickType_t) portMAX_DELAY);

	char message[20];

    for( ;; )
	{
    	/*Waits for change to the Lidar Sync GPIO (PB4)*/
    	xSemaphoreTake(xSemaphoreBiSync, ( TickType_t) portMAX_DELAY);

		//Create message
		sprintf(message, "%lu L_Sync\n", xTaskGetTickCount());

		xQueueSendToBack( xQueueUSART, &message, ( TickType_t ) portMAX_DELAY );


	}
}
/*-----------------------------------------------------------*/

static void prvSendMessage( void *pvParameters )
{
	char message[20];
    for( ;; )
	{
		/* Block until receive message. */
		xQueueReceive( xQueueUSART, &message, ( TickType_t ) portMAX_DELAY );

		HAL_UART_Transmit(&huart2, &message, sizeof( char [20] ), 100);


	}
}
/*-----------------------------------------------------------*/


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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	if (GPIO_Pin==GPIO_PIN_4){
		static BaseType_t pxHigherPriorityTaskWoken;

			xSemaphoreGiveFromISR( xSemaphoreBiSync, &pxHigherPriorityTaskWoken );

			if( pxHigherPriorityTaskWoken == pdTRUE )
				taskYIELD(); /* forces the context change */
	}
	if ((GPIO_Pin==GPIO_PIN_8)||(GPIO_Pin==GPIO_PIN_9)){
		static BaseType_t pxHigherPriorityTaskWoken;
			Wheel_L_Old = Wheel_L_New;
			xSemaphoreTakeFromISR(xSemaphoreBiEncoder, &pxHigherPriorityTaskWoken);
			Wheel_L_New = 2*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) + HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
			Wheel_L += QEM[Wheel_L_Old*4 + Wheel_L_New];
			xSemaphoreGiveFromISR( xSemaphoreBiEncoder, &pxHigherPriorityTaskWoken );

			if( pxHigherPriorityTaskWoken == pdTRUE )
				taskYIELD(); /* forces the context change */
	}
	if ((GPIO_Pin==GPIO_PIN_10)||(GPIO_Pin==GPIO_PIN_12)){
		static BaseType_t pxHigherPriorityTaskWoken;
			Wheel_R_Old = Wheel_R_New;
			xSemaphoreTakeFromISR(xSemaphoreBiEncoder, &pxHigherPriorityTaskWoken);
			Wheel_R_New = 2*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) + HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
			Wheel_R += QEM[Wheel_R_Old*4 + Wheel_R_New];
			xSemaphoreGiveFromISR( xSemaphoreBiEncoder, &pxHigherPriorityTaskWoken );

			if( pxHigherPriorityTaskWoken == pdTRUE )
				taskYIELD(); /* forces the context change */
	}
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


