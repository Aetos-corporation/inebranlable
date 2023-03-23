/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "trace.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId blinkTaskHandle;
uint32_t blinkTaskBuffer[ 128 ];
osStaticThreadDef_t blinkTaskControlBlock;
osThreadId xbeeTaskHandle;
uint32_t XbeeTaskBuffer[ 256 ];
osStaticThreadDef_t XbeeTaskControlBlock;
osMutexId traceMutexHandle;
osStaticMutexDef_t traceMutexControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBlinkTask(void const * argument);
extern void StartXbeeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of traceMutex */
  osMutexStaticDef(traceMutex, &traceMutexControlBlock);
  traceMutexHandle = osMutexCreate(osMutex(traceMutex));

  /* USER CODE BEGIN RTOS_MUTEX */

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
  /* definition and creation of blinkTask */
  osThreadStaticDef(blinkTask, StartBlinkTask, osPriorityIdle, 0, 128, blinkTaskBuffer, &blinkTaskControlBlock);
  blinkTaskHandle = osThreadCreate(osThread(blinkTask), NULL);

  /* definition and creation of xbeeTask */
  osThreadStaticDef(xbeeTask, StartXbeeTask, osPriorityNormal, 0, 256, XbeeTaskBuffer, &XbeeTaskControlBlock);
  xbeeTaskHandle = osThreadCreate(osThread(xbeeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  PRINT("\n\n\033[2J\033[H/---- StartUp ----/\n\n");
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartBlinkTask */
/**
  * @brief  Function implementing the blinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void const * argument)
{
  /* USER CODE BEGIN StartBlinkTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(100);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(100);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(100);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(1000);

//    uint8_t msg[] = "abcdefghijklmnopqrstuvwxyz\n\r";
//    LOG(msg, sizeof(msg)-1);
    static float f = 0;
    static int i = 0;
//    PRINT("LOG %d\n\r", i++);
    LOG_ROLL(f);
    f += 1;
  }
  /* USER CODE END StartBlinkTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
