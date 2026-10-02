/* Rate-control task interface and estimator states. */
#ifndef RATE_CONTROL_TASK_H
#define RATE_CONTROL_TASK_H

#include <stdint.h>
#include "estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RATE_EST_WAIT_SENSORS = 0,
    RATE_EST_COLLECT_STILL = 1,
    RATE_EST_RUNNING = 2,
    RATE_EST_FAULT = 3
} RateEstimator_State_t;

typedef enum
{
    RATE_EST_FAULT_NONE = 0,
    RATE_EST_FAULT_FLAGS = 1,
    RATE_EST_FAULT_IMU_INVALID = 2,
    RATE_EST_FAULT_GYRO_GAP = 3,
    RATE_EST_FAULT_GYRO_STALE = 4,
    RATE_EST_FAULT_ACCEL_STALE = 5,
    RATE_EST_FAULT_INIT = 6,
    RATE_EST_FAULT_PREDICT = 7,
    RATE_EST_FAULT_NUMERICAL = 8,
    RATE_EST_FAULT_OUTPUT = 9
} RateEstimator_Fault_t;

void RateControlTask_Run(void *argument);

/* Task context; signal after gyro publication and task creation. */
void RateControlTask_NotifyGyroReady(void);

#ifdef __cplusplus
}
#endif

#endif /* RATE_CONTROL_TASK_H */
