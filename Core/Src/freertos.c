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
#include <cmd_servo/cmd_servo.h>
#include <log/log.h>
#include <trace/trace.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_PWM_VALUE_SAFRAN  153
#define DEFAULT_PWM_VALUE_VOILE   150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for blinkTask */
osThreadId_t blinkTaskHandle;
uint32_t blinkTaskBuffer[ 128 ];
osStaticThreadDef_t blinkTaskControlBlock;
const osThreadAttr_t blinkTask_attributes = {
  .name = "blinkTask",
  .cb_mem = &blinkTaskControlBlock,
  .cb_size = sizeof(blinkTaskControlBlock),
  .stack_mem = &blinkTaskBuffer[0],
  .stack_size = sizeof(blinkTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWMTask */
osThreadId_t PWMTaskHandle;
uint32_t PWMTaskBuffer[ 256 ];
osStaticThreadDef_t PWMTaskControlBlock;
const osThreadAttr_t PWMTask_attributes = {
  .name = "PWMTask",
  .cb_mem = &PWMTaskControlBlock,
  .cb_size = sizeof(PWMTaskControlBlock),
  .stack_mem = &PWMTaskBuffer[0],
  .stack_size = sizeof(PWMTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xbeeTask */
osThreadId_t xbeeTaskHandle;
uint32_t XbeeTaskBuffer[ 1024 ];
osStaticThreadDef_t XbeeTaskControlBlock;
const osThreadAttr_t xbeeTask_attributes = {
  .name = "xbeeTask",
  .cb_mem = &XbeeTaskControlBlock,
  .cb_size = sizeof(XbeeTaskControlBlock),
  .stack_mem = &XbeeTaskBuffer[0],
  .stack_size = sizeof(XbeeTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
uint32_t gpsTaskBuffer[ 128 ];
osStaticThreadDef_t gpsTaskControlBlock;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .cb_mem = &gpsTaskControlBlock,
  .cb_size = sizeof(gpsTaskControlBlock),
  .stack_mem = &gpsTaskBuffer[0],
  .stack_size = sizeof(gpsTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for traceMutex */
osMutexId_t traceMutexHandle;
osStaticMutexDef_t traceMutexControlBlock;
const osMutexAttr_t traceMutex_attributes = {
  .name = "traceMutex",
  .cb_mem = &traceMutexControlBlock,
  .cb_size = sizeof(traceMutexControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBlinkTask(void *argument);
void startPWMTask(void *argument);
extern void StartXbeeTask(void *argument);
extern void StartGpsTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of traceMutex */
  traceMutexHandle = osMutexNew(&traceMutex_attributes);

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
  /* creation of blinkTask */
  blinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &blinkTask_attributes);

  /* creation of PWMTask */
  PWMTaskHandle = osThreadNew(startPWMTask, NULL, &PWMTask_attributes);

  /* creation of xbeeTask */
  xbeeTaskHandle = osThreadNew(StartXbeeTask, NULL, &xbeeTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBlinkTask */
/**
  * @brief  Function implementing the blinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void *argument)
{
  /* USER CODE BEGIN StartBlinkTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBlinkTask */
}

/* USER CODE BEGIN Header_startPWMTask */
/**
* @brief Function implementing the PWMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPWMTask */
void startPWMTask(void *argument)
{
  /* USER CODE BEGIN startPWMTask */
  /* Infinite loop */
	//démarrage des PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//initialisation des PWM dans les valeurs par défaut
	TIM1->CCR1 = DEFAULT_PWM_VALUE_SAFRAN; //safran à 0°
	TIM1->CCR4 = DEFAULT_PWM_VALUE_VOILE; //voile dans l'axe du bateau

	PRINT("PWM initialized\n");
  for(;;)
  {

	osDelay(100);
  }
  /* USER CODE END startPWMTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

