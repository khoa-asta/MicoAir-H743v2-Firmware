/*
 * imu_task.h
 *
 * Author: Viết Khoa
 */

#ifndef INC_IMU_TASK_H_
#define INC_IMU_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Calibrated sample in body FRD axes */
typedef struct
{
    float x;            /* Body X component. */
    float y;            /* Body Y component. */
    float z;            /* Body Z component. */
    uint32_t sequence;  /* Successful publication counter. */
    uint8_t valid;      /* Latest processing result. */
} IMUTask_Sample_t;

/* Process BMI088 data-ready and DMA completion events. */
void IMUTask_Run(void *argument);

/* Copy calibrated body FRD specific force in m/s^2. */
uint8_t IMUTask_GetAccSample(IMUTask_Sample_t *sample);

/* Copy calibrated body FRD angular rate in rad/s. */
uint8_t IMUTask_GetGyroSample(IMUTask_Sample_t *sample);

/* Signal accelerometer data ready. */
void IMUTask_NotifyAccDrdyFromISR(void);

/* Signal gyroscope data ready. */
void IMUTask_NotifyGyroDrdyFromISR(void);

/* Signal accelerometer DMA completion. */
void IMUTask_NotifyAccDmaDoneFromISR(void);

/* Signal gyroscope DMA completion. */
void IMUTask_NotifyGyroDmaDoneFromISR(void);

/* Signal an accelerometer DMA error. */
void IMUTask_NotifyAccDmaErrorFromISR(void);

/* Signal a gyroscope DMA error. */
void IMUTask_NotifyGyroDmaErrorFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_TASK_H_ */
