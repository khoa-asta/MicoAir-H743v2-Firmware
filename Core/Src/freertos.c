/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "imu_task.h"
#include "baro_task.h"
#include "compass_task.h"
#include "rc_task.h"
#include "rc_task_test.h"
#include "rc_load_test.h"
#include "rtos_timing.h"
#include "rate_control_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_FLAG_ACC_DRDY          (1UL << 0)
#define IMU_FLAG_GYRO_DRDY         (1UL << 1)
#define IMU_FLAG_ACC_DMA_DONE      (1UL << 2)
#define IMU_FLAG_GYRO_DMA_DONE     (1UL << 3)
#define IMU_FLAG_ACC_DMA_ERROR     (1UL << 4)
#define IMU_FLAG_GYRO_DMA_ERROR    (1UL << 5)

#define IMU_EVENT_MASK             (IMU_FLAG_ACC_DRDY      | \
                                    IMU_FLAG_GYRO_DRDY     | \
                                    IMU_FLAG_ACC_DMA_DONE  | \
                                    IMU_FLAG_GYRO_DMA_DONE | \
                                    IMU_FLAG_ACC_DMA_ERROR | \
                                    IMU_FLAG_GYRO_DMA_ERROR)

#define RC_LOAD_TEST_ENABLE 1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint32_t dbg_hal_tick = 0U;
volatile uint32_t dbg_rtos_tick = 0U;
volatile uint32_t dbg_task_count = 0U;
volatile uint32_t dbg_heap_free = 0U;
volatile uint32_t dbg_heap_min = 0U;
volatile uint32_t dbg_rtos_fault = 0U;
volatile uint32_t dbg_compass_task_count = 0U;

/* Minimum observed free stack space, in bytes. */
volatile uint32_t dbg_stack_default_min_bytes = 0xFFFFFFFFU;
volatile uint32_t dbg_stack_imu_min_bytes = 0xFFFFFFFFU;
volatile uint32_t dbg_stack_baro_min_bytes = 0xFFFFFFFFU;
volatile uint32_t dbg_stack_compass_min_bytes = 0xFFFFFFFFU;
volatile uint32_t dbg_stack_rc_min_bytes = 0xFFFFFFFFU;
volatile uint32_t dbg_stack_rate_min_bytes = 0xFFFFFFFFU;

volatile uint32_t dbg_stack_sample_count = 0U;

/* Definition for RCTask_test */
osThreadId_t RCTestTaskHandle;
const osThreadAttr_t RCTestTask_arrtibutes = {
		. name = "RCTestTask",
		.stack_size = 512 * 4,
		.priority = (osPriority_t)osPriorityBelowNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for BaroTask */
osThreadId_t BaroTaskHandle;
const osThreadAttr_t BaroTask_attributes = {
  .name = "BaroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CompassTask */
osThreadId_t CompassTaskHandle;
const osThreadAttr_t CompassTask_attributes = {
  .name = "CompassTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for RCTask */
osThreadId_t RCTaskHandle;
const osThreadAttr_t RCTask_attributes = {
  .name = "RCTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RateControlTask */
osThreadId_t RateControlTaskHandle;
const osThreadAttr_t RateControlTask_attributes = {
  .name = "RateControlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for i2c2Mutex */
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = {
  .name = "i2c2Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartIMUTask(void *argument);
void StartBaroTask(void *argument);
void StartCompassTask(void *argument);
void StartRCTask(void *argument);
void StartRateControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	/* USER CODE BEGIN vApplicationStackOverflowHook */

		(void)xTask;
	    (void)pcTaskName;

	    dbg_rtos_fault = 1U;

	    taskDISABLE_INTERRUPTS();

	    for (;;)
	    {
	        __NOP();
	    }

	/* USER CODE END vApplicationStackOverflowHook */

}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
    /* USER CODE BEGIN vApplicationMallocFailedHook */

    dbg_rtos_fault = 2U;

    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        __NOP();
    }

    /* USER CODE END vApplicationMallocFailedHook */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2c2Mutex */
  i2c2MutexHandle = osMutexNew(&i2c2Mutex_attributes);

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of BaroTask */
  BaroTaskHandle = osThreadNew(StartBaroTask, NULL, &BaroTask_attributes);

  /* creation of CompassTask */
  CompassTaskHandle = osThreadNew(StartCompassTask, NULL, &CompassTask_attributes);

  /* creation of RCTask */
  RCTaskHandle = osThreadNew(StartRCTask, NULL, &RCTask_attributes);

  /* creation of RateControlTask */
  RateControlTaskHandle = osThreadNew(StartRateControlTask, NULL, &RateControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//#if RC_LOAD_TEST_ENABLE
//  RCTestTaskHandle = osThreadNew(RCLoadTest_Run,
//                                NULL,
//                                &RCTestTask_arrtibutes);
//#else
//  RCTestTaskHandle = osThreadNew(RCTaskTest_Run,
//                                NULL,
//                                &RCTestTask_arrtibutes);
//#endif
//
//  if (RCTestTaskHandle == NULL)
//  {
//      Error_Handler();
//  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	(void)argument;

	/* Verify persistent task handles before querying their stacks. */
	if ((IMUTaskHandle == NULL) ||
	    (BaroTaskHandle == NULL) ||
	    (CompassTaskHandle == NULL) ||
	    (RCTaskHandle == NULL))
	{
	    Error_Handler();
	}

	for (;;)
	{
	    dbg_hal_tick = HAL_GetTick();
	    dbg_rtos_tick = osKernelGetTickCount();
	    dbg_task_count = uxTaskGetNumberOfTasks();

	    dbg_heap_free = xPortGetFreeHeapSize();
	    dbg_heap_min = xPortGetMinimumEverFreeHeapSize();

	    /* CMSIS-RTOS2 returns stack space in bytes. */
	    dbg_stack_default_min_bytes =
	        osThreadGetStackSpace(osThreadGetId());

	    dbg_stack_imu_min_bytes =
	        osThreadGetStackSpace(IMUTaskHandle);

	    dbg_stack_baro_min_bytes =
	        osThreadGetStackSpace(BaroTaskHandle);

	    dbg_stack_compass_min_bytes =
	        osThreadGetStackSpace(CompassTaskHandle);

	    dbg_stack_rc_min_bytes =
	        osThreadGetStackSpace(RCTaskHandle);
	    dbg_stack_rate_min_bytes = osThreadGetStackSpace(RateControlTaskHandle);

	    dbg_stack_sample_count++;

	    /* One second at the project's 1000 Hz kernel tick. */
	    osDelay(1000U);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  IMUTask_Run(argument);
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
* @brief Function implementing the BaroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void *argument)
{
  /* USER CODE BEGIN StartBaroTask */
  /* Infinite loop */
  BaroTask_Run(argument);
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartCompassTask */
/**
* @brief Function implementing the CompassTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCompassTask */
void StartCompassTask(void *argument)
{
  /* USER CODE BEGIN StartCompassTask */
  /* Infinite loop */
  CompassTask_Run(argument);
  /* USER CODE END StartCompassTask */
}

/* USER CODE BEGIN Header_StartRCTask */
/**
* @brief Function implementing the RCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRCTask */
void StartRCTask(void *argument)
{
  /* USER CODE BEGIN StartRCTask */
  /* Infinite loop */
  RCTask_Run(argument);
  /* USER CODE END StartRCTask */
}

/* USER CODE BEGIN Header_StartRateControlTask */
/**
* @brief Function implementing the RateControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRateControlTask */
void StartRateControlTask(void *argument)
{
  /* USER CODE BEGIN StartRateControlTask */
  /* Infinite loop */
  RateControlTask_Run(argument);
  /* USER CODE END StartRateControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

