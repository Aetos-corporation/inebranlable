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
#include <imu/imu.h>
#include <gps/gps_zed_f9p.h>
#include <system/system.h>
#include <windSensor/windSensor.h>
#include <navigation/navigation.h>
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
uint32_t blinkTaskBuffer[ 1024 ];
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
uint32_t PWMTaskBuffer[ 1024 ];
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
uint32_t gpsTaskBuffer[ 1024 ];
osStaticThreadDef_t gpsTaskControlBlock;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .cb_mem = &gpsTaskControlBlock,
  .cb_size = sizeof(gpsTaskControlBlock),
  .stack_mem = &gpsTaskBuffer[0],
  .stack_size = sizeof(gpsTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
uint32_t imuTaskBuffer[ 1024 ];
osStaticThreadDef_t imuTaskControlBlock;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .cb_mem = &imuTaskControlBlock,
  .cb_size = sizeof(imuTaskControlBlock),
  .stack_mem = &imuTaskBuffer[0],
  .stack_size = sizeof(imuTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for navTask */
osThreadId_t navTaskHandle;
uint32_t navTaskBuffer[ 1024 ];
osStaticThreadDef_t navTaskControlBlock;
const osThreadAttr_t navTask_attributes = {
  .name = "navTask",
  .cb_mem = &navTaskControlBlock,
  .cb_size = sizeof(navTaskControlBlock),
  .stack_mem = &navTaskBuffer[0],
  .stack_size = sizeof(navTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for windSensorTask */
osThreadId_t windSensorTaskHandle;
uint32_t windSensorTaskBuffer[ 1024 ];
osStaticThreadDef_t windSensorTaskControlBlock;
const osThreadAttr_t windSensorTask_attributes = {
  .name = "windSensorTask",
  .cb_mem = &windSensorTaskControlBlock,
  .cb_size = sizeof(windSensorTaskControlBlock),
  .stack_mem = &windSensorTaskBuffer[0],
  .stack_size = sizeof(windSensorTaskBuffer),
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
void navTask_loop(void);
void demo_loop(void);
void debug_loop(void);
/* USER CODE END FunctionPrototypes */

void StartBlinkTask(void *argument);
extern void startPWMTask(void *argument);
extern void StartXbeeTask(void *argument);
extern void StartGpsTask(void *argument);
extern void StartImuTask(void *argument);
void StartNavTask(void *argument);
extern void StartWindSensorTask(void *argument);

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

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of navTask */
  navTaskHandle = osThreadNew(StartNavTask, NULL, &navTask_attributes);

  /* creation of windSensorTask */
  windSensorTaskHandle = osThreadNew(StartWindSensorTask, NULL, &windSensorTask_attributes);

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
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	osDelay(100);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	osDelay(100);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	osDelay(100);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	osDelay(1000);
  }
  /* USER CODE END StartBlinkTask */
}

/* USER CODE BEGIN Header_StartNavTask */
/**
* @brief Function implementing the navTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNavTask */
void StartNavTask(void *argument)
{
  /* USER CODE BEGIN StartNavTask */
	while( !sys_testInitFlag(SYS_MASK_XBEE) && !sys_testInitFlag(SYS_MASK_IMU))
		osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
//	  debug_loop();
	  navTask_loop();
  }
  /* USER CODE END StartNavTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void navTask_loop(void)
{
	struct point bouee;
	bouee.x = 47.2524959;
	bouee.y = -1.3724528;

	struct point mat;
	mat.x = 0.0;
	mat.y = 0.0;

	// Update time
	LOG_INFO("update (%ds)", HAL_GetTick() /1000 );

	// Log mode de nav
	static uint8_t navMode;
	uint8_t temp = HAL_GPIO_ReadPin(AUTO_GPIO_Port, AUTO_Pin);
	if(temp != navMode){
		// nav mode toggled
		navMode = temp;  // update val
		// Log
		if(navMode)
			LOG_INFO("Mode Auto ON");
		else {
			LOG_INFO("Mode Auto OFF");
			return;
		}
	}

	// Get IMU
	float yaw = (float) imu_getYaw();
	LOG_YAW(yaw);
	PRINT("Get imu OK");

	// Get Girouette
	float windAngle = getWindAngle();
	PRINT("Get girouette OK");

	// Get GPS
	mat.x = GPS_getLatitude();
	mat.y = GPS_getLongitude();
	PRINT("Get gps OK");

	// Navigation
	PRINT("Nav done");
	int strategie = decision_strategie(bouee, mat, windAngle, yaw);
	PRINT("  Strategie: %d", strategie);

	struct point navigation_result = navigation(bouee, mat, strategie, windAngle, yaw);
	PRINT("  Navigation:   - latitude: %f     - longitude: %f", navigation_result.x, navigation_result.y);

	struct cmd pilotage_result = pilotage(navigation_result, mat, windAngle, yaw);
	PRINT("  Commandes servos:   - Safran: %f     - Voile: %f", pilotage_result.safran, pilotage_result.voile);

	// Commandes servos
	set_PWM_value_safran(pilotage_result.safran);
	set_PWM_value_voile(pilotage_result.voile);
	LOG_SAFRAN(pilotage_result.safran);
	LOG_VOILE(pilotage_result.voile);
	PRINT("Commandes servo done");

	PRINT(" ");  //newline
	osDelay(1000 / portTICK_PERIOD_MS);
}

void demo_loop(void)
{
	struct point mat;
	mat.x = 0.0;
	mat.y = 0.0;

	// Update time
	LOG_INFO("Update (%ds)", HAL_GetTick() /1000 );

	// Get IMU
	float yaw = (float) imu_getYaw();
	PRINT("imu = %f", yaw);

	// Get Girouette
	float windAngle = getWindAngle();
	PRINT("girouette = %f", windAngle);

	// Get GPS
	mat.x = GPS_getLatitude();
	mat.y = GPS_getLongitude();
	PRINT("GPS:   - latitude: %f   - longitude: %f", mat.x, mat.y);

	// Commandes servos
	static bool toggle = 0;
	if(toggle){
		set_PWM_value_safran(-30);
		set_PWM_value_voile(0);
		LOG_SAFRAN(-30);
		LOG_VOILE(0);
	} else {
		set_PWM_value_voile(90);
		set_PWM_value_safran(30);
		LOG_SAFRAN(90);
		LOG_VOILE(30);
	}
	toggle = !toggle;

	// Log mode de nav
	static uint8_t navMode;
	uint8_t temp = HAL_GPIO_ReadPin(AUTO_GPIO_Port, AUTO_Pin);
	if(temp != navMode){
		// nav mode toggled
		navMode = temp;  // update val
		// Log
		if(navMode)
			LOG_INFO("Mode Auto ON");
		else
			LOG_INFO("Mode Auto OFF");
	}

	PRINT(" ");  //newline
	osDelay(3000 / portTICK_PERIOD_MS);
}

void debug_loop(){
	TIM1->CCR4 = 135;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 140;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 145;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 150;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 155;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 160;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 165;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 170;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 175;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 180;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 185;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 190;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 195;
	osDelay(3000 / portTICK_PERIOD_MS);
	TIM1->CCR4 = 200;
	osDelay(3000 / portTICK_PERIOD_MS);
}
/* USER CODE END Application */

