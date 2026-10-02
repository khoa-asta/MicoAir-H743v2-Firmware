/*
 * compass_task.h
 *
 * Author: Viet Khoa
 */

#ifndef COMPASS_TASK_H
#define COMPASS_TASK_H

#include "qmc5883l.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Latest magnetometer publication */
typedef struct
{
    QMC5883L_Data_t data;   /* Sensor values. */
    uint32_t sequence;      /* Valid publication counter. */
    uint32_t publish_tick;  /* RTOS publication time. */
    uint8_t valid;          /* Published sample availability. */
} CompassTask_Sample_t;

/* Read and publish QMC5883L samples at the configured rate. */
void CompassTask_Run(void *argument);

/* Copy the latest compass publication in task context. */
uint8_t CompassTask_GetSample(CompassTask_Sample_t *sample);

#ifdef __cplusplus
}
#endif
#endif /* COMPASS_TASK_H */
