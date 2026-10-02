/*
 * compass_task.c
 *
 * Author: Viet Khoa
 */

/* Includes */
#include "compass_task.h"
#include "qmc5883l.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>

/* Task configuration */
#define COMPASS_TASK_RATE_HZ             50U
#define COMPASS_MUTEX_TIMEOUT_TICKS      5U

/* RTOS objects */
extern osMutexId_t i2c2MutexHandle;

/* Published sensor data */
static CompassTask_Sample_t latest_sample;

/* Publish a finite magnetometer sample. */
static void CompassTask_Publish(const QMC5883L_Data_t *data)
{
    if (!(isfinite(data->x_uT) && isfinite(data->y_uT) && isfinite(data->z_uT)))
    {
        return;
    }

    const uint32_t tick = osKernelGetTickCount();
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    latest_sample.data = *data;
    latest_sample.sequence++;
    latest_sample.publish_tick = tick;
    latest_sample.valid = 1U;

    __DMB();
    __set_PRIMASK(primask);
}

/* Copy the latest compass publication in task context. */
uint8_t CompassTask_GetSample(CompassTask_Sample_t *sample)
{
    if (sample == NULL)
    {
        return 0U;
    }

    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    *sample = latest_sample;

    __DMB();
    __set_PRIMASK(primask);

    return sample->valid;
}

/* Read and publish QMC5883L samples at the configured rate. */
void CompassTask_Run(void *argument)
{
    QMC5883L_Data_t data;
    HAL_StatusTypeDef status;

    (void)argument;

    const uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t period_ticks = tick_freq / COMPASS_TASK_RATE_HZ;

    if (period_ticks == 0U)
    {
        period_ticks = 1U;
    }

    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
        if (osMutexAcquire(i2c2MutexHandle, COMPASS_MUTEX_TIMEOUT_TICKS) == osOK)
        {
            status = QMC5883L_Read(&data);
            (void)osMutexRelease(i2c2MutexHandle);

            if (status == HAL_OK)
            {
                CompassTask_Publish(&data);
            }
        }

        next_wake += period_ticks;
        (void)osDelayUntil(next_wake);
    }
}
