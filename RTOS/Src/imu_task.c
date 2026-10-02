/*
 * imu_task.c
 *
 * Author: Viết Khoa
 */

/* Includes */
#include "imu_task.h"
#include "bmi088.h"
#include "imu_calibration.h"
#include "rate_control_task.h"
#include "cmsis_os2.h"
#include <stddef.h>

/* Task configuration */
#define IMU_FLAG_ACC_DRDY          (1UL << 0)
#define IMU_FLAG_GYRO_DRDY         (1UL << 1)
#define IMU_FLAG_ACC_DMA_DONE      (1UL << 2)
#define IMU_FLAG_GYRO_DMA_DONE     (1UL << 3)
#define IMU_FLAG_ACC_DMA_ERROR     (1UL << 4)
#define IMU_FLAG_GYRO_DMA_ERROR    (1UL << 5)

#define IMU_EVENT_MASK             (IMU_FLAG_ACC_DRDY | IMU_FLAG_GYRO_DRDY | \
                                    IMU_FLAG_ACC_DMA_DONE | IMU_FLAG_GYRO_DMA_DONE | \
                                    IMU_FLAG_ACC_DMA_ERROR | IMU_FLAG_GYRO_DMA_ERROR)

/* RTOS objects */
extern osThreadId_t IMUTaskHandle;

/* Acquisition state */
static volatile uint8_t imu_task_ready = 0U;
static uint8_t imu_acc_request = 0U;
static uint8_t imu_gyro_request = 0U;

static IMUTask_Sample_t imu_acc_sample = {0};
static IMUTask_Sample_t imu_gyro_sample = {0};

/* Raw and converted sensor-axis data */
BMI088_RawData_t g_imu_acc_raw = {0};
BMI088_RawData_t g_imu_gyro_raw = {0};
BMI088_Data_t g_imu_acc_g = {0};
BMI088_Data_t g_imu_gyro_dps = {0};

/* Private function prototypes */
static void IMUTask_SetFlagFromISR(uint32_t flag);
static void IMUTask_StartPendingTransfer(void);
static void IMUTask_ProcessAccSample(void);
static void IMUTask_ProcessGyroSample(void);
static void IMUTask_PublishSample(IMUTask_Sample_t *sample, const float value[3]);
static uint8_t IMUTask_CopySample(const IMUTask_Sample_t *source, IMUTask_Sample_t *sample);

/* Publish calibrated XYZ values or invalidate the sample. */
static void IMUTask_PublishSample(IMUTask_Sample_t *sample, const float value[3])
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    if (value != NULL)
    {
        sample->x = value[0];
        sample->y = value[1];
        sample->z = value[2];
        sample->sequence++;
        sample->valid = 1U;
    }
    else
    {
        /* Invalidate the retained XYZ values. */
        sample->valid = 0U;
    }

    __DMB();
    __set_PRIMASK(primask);
}

/* Copy XYZ, sequence and validity as one snapshot. */
static uint8_t IMUTask_CopySample(const IMUTask_Sample_t *source, IMUTask_Sample_t *sample)
{
    uint32_t primask;

    if (sample == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();

    *sample = *source;

    __DMB();
    __set_PRIMASK(primask);

    return sample->valid;
}

/* Convert and calibrate a completed accelerometer sample. */
static void IMUTask_ProcessAccSample(void)
{
    float sensor_g[3];
    float body_mps2[3];

    if (BMI088_Accel_ReadRaw_DMA_Get(&g_imu_acc_raw) != HAL_OK)
    {
        IMUTask_PublishSample(&imu_acc_sample, NULL);

        return;
    }

    BMI088_Accel_ConvertToG(&g_imu_acc_raw, &g_imu_acc_g);

    sensor_g[0] = g_imu_acc_g.x;
    sensor_g[1] = g_imu_acc_g.y;
    sensor_g[2] = g_imu_acc_g.z;

    /* Apply SI conversion, body mapping and calibration. */
    if (!IMU_Calibration_ApplyAccelG(sensor_g, body_mps2))
    {
        IMUTask_PublishSample(&imu_acc_sample, NULL);

        return;
    }

    IMUTask_PublishSample(&imu_acc_sample, body_mps2);
}

/* Convert and calibrate a completed gyroscope sample. */
static void IMUTask_ProcessGyroSample(void)
{
    float sensor_dps[3];
    float body_rad_s[3];

    if (BMI088_Gyro_ReadRaw_DMA_Get(&g_imu_gyro_raw) != HAL_OK)
    {
        IMUTask_PublishSample(&imu_gyro_sample, NULL);

        return;
    }

    BMI088_Gyro_ConvertToDps(&g_imu_gyro_raw, &g_imu_gyro_dps);

    sensor_dps[0] = g_imu_gyro_dps.x;
    sensor_dps[1] = g_imu_gyro_dps.y;
    sensor_dps[2] = g_imu_gyro_dps.z;

    /* Apply SI conversion, body mapping and calibration. */
    if (!IMU_Calibration_ApplyGyroDps(sensor_dps, body_rad_s))
    {
        IMUTask_PublishSample(&imu_gyro_sample, NULL);

        return;
    }

    IMUTask_PublishSample(&imu_gyro_sample, body_rad_s);
    RateControlTask_NotifyGyroReady();
}

/* Set an IMU event flag from interrupt context. */
static void IMUTask_SetFlagFromISR(uint32_t flag)
{
    if ((imu_task_ready == 0U) || (IMUTaskHandle == NULL))
    {
        return;
    }

    (void)osThreadFlagsSet(IMUTaskHandle, flag);
}

/* Start a pending DMA transfer with gyroscope priority. */
static void IMUTask_StartPendingTransfer(void)
{
    HAL_StatusTypeDef status;

    if (BMI088_SPI_IsBusy() != 0U)
    {
        return;
    }

    if (imu_gyro_request != 0U)
    {
        status = BMI088_Gyro_ReadRaw_DMA_Start();

        if (status == HAL_OK)
        {
            imu_gyro_request = 0U;
        }
        else if (status != HAL_BUSY)
        {
            imu_gyro_request = 0U;
            IMUTask_PublishSample(&imu_gyro_sample, NULL);
        }

        return;
    }

    if (imu_acc_request != 0U)
    {
        status = BMI088_Accel_ReadRaw_DMA_Start();

        if (status == HAL_OK)
        {
            imu_acc_request = 0U;
        }
        else if (status != HAL_BUSY)
        {
            imu_acc_request = 0U;
            IMUTask_PublishSample(&imu_acc_sample, NULL);
        }
    }
}

/* Process BMI088 data-ready and DMA completion events. */
void IMUTask_Run(void *argument)
{
    uint32_t flags;

    (void)argument;

    imu_acc_request = 0U;
    imu_gyro_request = 0U;
    IMUTask_PublishSample(&imu_acc_sample, NULL);
    IMUTask_PublishSample(&imu_gyro_sample, NULL);
    imu_task_ready = 1U;

    for (;;)
    {
        flags = osThreadFlagsWait(IMU_EVENT_MASK, osFlagsWaitAny, osWaitForever);

        if ((flags & osFlagsError) != 0U)
        {
            IMUTask_PublishSample(&imu_acc_sample, NULL);
            IMUTask_PublishSample(&imu_gyro_sample, NULL);
            continue;
        }

        if ((flags & IMU_FLAG_ACC_DRDY) != 0U)
        {
            imu_acc_request = 1U;
        }

        if ((flags & IMU_FLAG_GYRO_DRDY) != 0U)
        {
            imu_gyro_request = 1U;
        }

        /* DMA errors take precedence over completion events. */
        if ((flags & IMU_FLAG_ACC_DMA_ERROR) != 0U)
        {
            if ((flags & IMU_FLAG_ACC_DMA_DONE) != 0U)
            {
                (void)BMI088_Accel_ReadRaw_DMA_Get(&g_imu_acc_raw);
            }

            IMUTask_PublishSample(&imu_acc_sample, NULL);
        }
        else if ((flags & IMU_FLAG_ACC_DMA_DONE) != 0U)
        {
            IMUTask_ProcessAccSample();
        }

        if ((flags & IMU_FLAG_GYRO_DMA_ERROR) != 0U)
        {
            if ((flags & IMU_FLAG_GYRO_DMA_DONE) != 0U)
            {
                (void)BMI088_Gyro_ReadRaw_DMA_Get(&g_imu_gyro_raw);
            }

            IMUTask_PublishSample(&imu_gyro_sample, NULL);
        }
        else if ((flags & IMU_FLAG_GYRO_DMA_DONE) != 0U)
        {
            IMUTask_ProcessGyroSample();
        }

        IMUTask_StartPendingTransfer();
    }
}

/* Copy calibrated body FRD specific force in m/s^2. */
uint8_t IMUTask_GetAccSample(IMUTask_Sample_t *sample)
{
    return IMUTask_CopySample(&imu_acc_sample, sample);
}

/* Copy calibrated body FRD angular rate in rad/s. */
uint8_t IMUTask_GetGyroSample(IMUTask_Sample_t *sample)
{
    return IMUTask_CopySample(&imu_gyro_sample, sample);
}

/* Signal accelerometer data ready. */
void IMUTask_NotifyAccDrdyFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_ACC_DRDY);
}

/* Signal gyroscope data ready. */
void IMUTask_NotifyGyroDrdyFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_GYRO_DRDY);
}

/* Signal accelerometer DMA completion. */
void IMUTask_NotifyAccDmaDoneFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_ACC_DMA_DONE);
}

/* Signal gyroscope DMA completion. */
void IMUTask_NotifyGyroDmaDoneFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_GYRO_DMA_DONE);
}

/* Signal an accelerometer DMA error. */
void IMUTask_NotifyAccDmaErrorFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_ACC_DMA_ERROR);
}

/* Signal a gyroscope DMA error. */
void IMUTask_NotifyGyroDmaErrorFromISR(void)
{
    IMUTask_SetFlagFromISR(IMU_FLAG_GYRO_DMA_ERROR);
}
