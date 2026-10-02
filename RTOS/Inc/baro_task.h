/*
 * baro_task.h
 *
 * Author: Viet Khoa
 */

#ifndef BARO_TASK_H
#define BARO_TASK_H

#include "spl06.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Latest barometer publication */
typedef struct
{
    SPL06_Data_t data;      /* Sensor values. */
    uint32_t sequence;      /* Valid publication counter. */
    uint32_t publish_tick;  /* RTOS publication time. */
    uint8_t valid;          /* Published sample availability. */
} BaroTask_Sample_t;

/* Read and publish SPL06 samples at the configured rate. */
void BaroTask_Run(void *argument);

/* Copy the latest barometer publication in task context. */
uint8_t BaroTask_GetSample(BaroTask_Sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* BARO_TASK_H */
