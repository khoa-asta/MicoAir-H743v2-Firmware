/*
 * baro_task.c
 *
 * Author: Viet Khoa
 */

/* Includes */
#include "baro_task.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>

/* Task configuration */
#define BARO_TASK_RATE_HZ             50U
#define BARO_MUTEX_TIMEOUT_TICKS      5U
#define BARO_MIN_PRESSURE_PA          30000.0f
#define BARO_MAX_PRESSURE_PA          120000.0f

/* RTOS objects */
extern osMutexId_t i2c2MutexHandle;

/* Published sensor data */
static BaroTask_Sample_t s_latest_sample;
static float s_reference_pressure_pa;

/* Validate and publish pressure and temperature. */
static void BaroTask_Publish(const SPL06_Data_t *data)
{
    if (!isfinite(data->pressure_pa) ||
        data->pressure_pa < BARO_MIN_PRESSURE_PA ||
        data->pressure_pa > BARO_MAX_PRESSURE_PA ||
        !isfinite(data->temperature_c))
    {
        return;
    }

    float reference;

    if (s_reference_pressure_pa > 0.0f)
    {
        reference = s_reference_pressure_pa;
    }
    else
    {
        reference = data->pressure_pa;
    }

    const float altitude = SPL06_PressureToAltitude(data->pressure_pa, reference);

    if (!isfinite(altitude))
    {
        return;
    }

    s_reference_pressure_pa = reference;

    /* Capture the publication time. */
    const uint32_t tick = osKernelGetTickCount();
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    s_latest_sample.data = *data;
    s_latest_sample.sequence++;
    s_latest_sample.publish_tick = tick;
    s_latest_sample.valid = 1U;

    __DMB();
    __set_PRIMASK(primask);
}

/* Copy the latest barometer publication in task context. */
uint8_t BaroTask_GetSample(BaroTask_Sample_t *sample)
{
    if (sample == NULL)
    {
        return 0U;
    }

    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    *sample = s_latest_sample;

    __DMB();
    __set_PRIMASK(primask);

    return sample->valid;
}

/* Read and publish SPL06 samples at the configured rate. */
void BaroTask_Run(void *argument)
{
    (void)argument;

    const uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t period_ticks = (uint32_t)
        (((uint64_t)tick_freq + BARO_TASK_RATE_HZ - 1U) / BARO_TASK_RATE_HZ);

    if (period_ticks == 0U)
    {
        period_ticks = 1U;
    }

    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
        if (i2c2MutexHandle != NULL &&
            osMutexAcquire(i2c2MutexHandle, BARO_MUTEX_TIMEOUT_TICKS) == osOK)
        {
            SPL06_Data_t data = {NAN, NAN};
            const HAL_StatusTypeDef status = SPL06_Read(&data);
            const osStatus_t release_status = osMutexRelease(i2c2MutexHandle);

            if (release_status == osOK && status == HAL_OK)
            {
                BaroTask_Publish(&data);
            }
        }

        /* Advance the periodic deadline. */
        next_wake += period_ticks;
        const uint32_t now = osKernelGetTickCount();
        const uint32_t remaining = (uint32_t)(next_wake - now);

        if (remaining == 0U || remaining > INT32_MAX)
        {
            next_wake = now + period_ticks;
        }

        if (osDelayUntil(next_wake) != osOK)
        {
            (void)osDelay(period_ticks);
            next_wake = osKernelGetTickCount();
        }
    }
}
