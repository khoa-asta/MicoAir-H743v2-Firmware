#include "rate_control_task.h"
#include "rate_estimator_config.h"
#include "estimator_port.h"
#include "imu_task.h"
#include "baro_task.h"
#include "compass_task.h"
#include "cmsis_os2.h"
#include "flight_control.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RATE_FLAG_GYRO_READY (1UL << 0)
#define GRAVITY_MPS2 9.80665f

extern osThreadId_t RateControlTaskHandle;

/* Estimator storage. */
static EstimatorPort_t s_port;
static Estimator_Output_t s_output;

/* Estimator port configuration. */
static const EstimatorPort_Config_t s_port_config = {
    .mag_calibrated = RATE_EST_MAG_CALIBRATED != 0U, /* Magnetometer calibration status. */
    .mag_offset_ut = RATE_EST_MAG_OFFSET_UT,         /* Magnetometer hard-iron offset. */
    .mag_to_body = RATE_EST_MAG_TO_BODY              /* Magnetometer-to-body rotation matrix. */
};

/* Running statistics for a 3-axis vector. */
typedef struct
{
    uint32_t count; /* Number of accumulated samples. */
    float mean[3];  /* Running mean of X, Y and Z axes. */
    float m2[3];    /* Running squared deviation for variance. */
} VectorStats_t;

/* Startup sensor statistics and references. */
static struct
{
    VectorStats_t gyro;       /* Gyroscope statistics during startup. */
    VectorStats_t accel;      /* Accelerometer statistics during startup. */
    VectorStats_t mag;        /* Magnetometer statistics during startup. */
    float accel_anchor[3];    /* Initial acceleration reference for motion detection. */
    float pressure_mean;      /* Mean barometric pressure during startup. */
    uint32_t pressure_count;  /* Number of accumulated barometer samples. */
    uint32_t start_tick;      /* Tick when still-sample collection started. */
    uint32_t accel_sequence;  /* Last collected accelerometer sequence. */
    uint32_t baro_sequence;   /* Last collected barometer sequence. */
    uint32_t mag_sequence;    /* Last collected magnetometer sequence. */
    bool have_accel;          /* Accelerometer sample has been collected. */
    bool have_baro;           /* Barometer sample has been collected. */
    bool have_mag;            /* Magnetometer sample has been collected. */
} s_startup;

/* Estimator runtime state. */
static struct
{
    RateEstimator_State_t state; /* Current estimator state. */
    RateEstimator_Fault_t fault_reason; /* First latched estimator fault. */
    uint32_t tick_freq;          /* RTOS tick frequency in Hz. */
    uint32_t imu_timeout;        /* Maximum allowed IMU sample age in ticks. */
    uint32_t aid_timeout;        /* Maximum allowed aiding sample age in ticks. */
    uint32_t boot_tick;          /* Tick captured when the task starts. */
    uint32_t gyro_sequence;      /* Last processed gyroscope sequence. */
    uint32_t accel_sequence;     /* Last observed accelerometer sequence. */
    uint32_t gyro_tick;          /* Tick when a new gyro publication was observed. */
    uint32_t accel_tick;         /* Tick when a new accel publication was observed. */
    uint32_t baro_sequence;      /* Last consumed barometer sequence. */
    uint32_t mag_sequence;       /* Last consumed magnetometer sequence. */
    bool gyro_stale;             /* Gyroscope observation has expired. */
    bool baro_fresh;             /* Barometer sample is usable. */
    bool mag_fresh;              /* Magnetometer sample is usable. */
    bool have_gyro;              /* At least one valid gyroscope sample received. */
    bool have_accel;             /* At least one valid accelerometer sample received. */
    bool have_baro;              /* At least one barometer sample consumed. */
    bool have_mag;               /* At least one magnetometer sample consumed. */
} s_runtime = {.gyro_stale = true};

/* Convert milliseconds to RTOS ticks. */
static uint32_t MillisecondsToTicks(uint32_t milliseconds)
{
    const uint32_t ticks = (uint32_t)(((uint64_t)milliseconds * s_runtime.tick_freq + 999U) / 1000U);

    if (ticks == 0U)
    {
        return 1U;
    }

    return ticks;
}

/* Calculate the magnitude of a 3-axis IMU sample. */
static float SampleNorm(const IMUTask_Sample_t *sample)
{
    return sqrtf(sample->x * sample->x + sample->y * sample->y + sample->z * sample->z);
}

/* Check IMU sample validity and finite values. */
static bool ImuFinite(const IMUTask_Sample_t *sample)
{
    return sample->valid != 0U && isfinite(sample->x) && isfinite(sample->y) && isfinite(sample->z);
}

/* Add one 3-axis sample to the running statistics. */
static void StatsAdd(VectorStats_t *stats, float x, float y, float z)
{
    const float value[3] = {x, y, z};

    stats->count++;

    for (int i = 0; i < 3; ++i)
    {
        const float delta = value[i] - stats->mean[i];
        stats->mean[i] += delta / (float)stats->count;
        stats->m2[i] += delta * (value[i] - stats->mean[i]);
    }
}

/* Check 3-axis sample variance against the startup threshold. */
static bool StatsQuiet(const VectorStats_t *stats, float max_std)
{
    if (stats->count < 2U)
    {
        return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        if (!isfinite(stats->m2[i]) || stats->m2[i] / (float)(stats->count - 1U) > max_std * max_std)
        {
            return false;
        }
    }

    return true;
}

/* Update estimator state. */
static void SetState(RateEstimator_State_t state)
{
    s_runtime.state = state;
}

/* Enter estimator fault state. */
static void Fault(RateEstimator_Fault_t reason)
{
    if (s_runtime.state == RATE_EST_FAULT)
    {
        return;
    }

    SetState(RATE_EST_FAULT);
    s_runtime.fault_reason = reason;

    if (s_port.filter.initialized)
    {
        s_port.needs_resync = true;
    }
}

/* Reset startup sample collection. */
static void ResetCollection(void)
{
    if (s_runtime.state != RATE_EST_COLLECT_STILL)
    {
        return;
    }

    memset(&s_startup, 0, sizeof(s_startup));

    SetState(RATE_EST_WAIT_SENSORS);
}

/* Check IMU observation ages and latch stale faults. */
static void CheckImuAges(uint32_t now)
{
    const bool accel_stale = !s_runtime.have_accel ||
        (uint32_t)(now - s_runtime.accel_tick) >= s_runtime.imu_timeout;

    s_runtime.gyro_stale = !s_runtime.have_gyro ||
        (uint32_t)(now - s_runtime.gyro_tick) >= s_runtime.imu_timeout;

    if (s_runtime.gyro_stale || accel_stale)
    {
        if (s_runtime.state == RATE_EST_RUNNING)
        {
            Fault(s_runtime.gyro_stale ? RATE_EST_FAULT_GYRO_STALE : RATE_EST_FAULT_ACCEL_STALE);
        }
        else
        {
            ResetCollection();
        }
    }
}

/* Read aiding samples and check publication ages. */
static void ReadAiding(uint32_t now, BaroTask_Sample_t *baro, CompassTask_Sample_t *mag)
{
    const bool baro_valid = BaroTask_GetSample(baro) != 0U;
    const bool mag_valid = CompassTask_GetSample(mag) != 0U;

    s_runtime.baro_fresh = baro_valid &&
        (uint32_t)(now - baro->publish_tick) < s_runtime.aid_timeout &&
        isfinite(baro->data.pressure_pa) &&
        baro->data.pressure_pa >= 30000.0f && baro->data.pressure_pa <= 120000.0f;

    s_runtime.mag_fresh = mag_valid &&
        (uint32_t)(now - mag->publish_tick) < s_runtime.aid_timeout &&
        isfinite(mag->data.x_uT) && isfinite(mag->data.y_uT) && isfinite(mag->data.z_uT);
}

/* Collect stationary startup samples and initialize the estimator. */
static void CollectStartup(uint32_t now, const IMUTask_Sample_t *gyro, const IMUTask_Sample_t *accel, const BaroTask_Sample_t *baro, const CompassTask_Sample_t *mag)
{
    if (!s_runtime.baro_fresh || (s_port_config.mag_calibrated && !s_runtime.mag_fresh))
    {
        ResetCollection();
        return;
    }

    if (SampleNorm(gyro) > RATE_EST_START_GYRO_MAX || fabsf(SampleNorm(accel) - GRAVITY_MPS2) > RATE_EST_START_ACCEL_GATE)
    {
        ResetCollection();
        return;
    }

    if (s_runtime.state == RATE_EST_WAIT_SENSORS)
    {
        if ((uint32_t)(now - s_runtime.boot_tick) < MillisecondsToTicks(RATE_EST_SETTLE_MS))
        {
            return;
        }

        memset(&s_startup, 0, sizeof(s_startup));

        s_startup.start_tick = now;
        s_startup.accel_anchor[0] = accel->x;
        s_startup.accel_anchor[1] = accel->y;
        s_startup.accel_anchor[2] = accel->z;

        SetState(RATE_EST_COLLECT_STILL);
    }

    const float dx = accel->x - s_startup.accel_anchor[0];
    const float dy = accel->y - s_startup.accel_anchor[1];
    const float dz = accel->z - s_startup.accel_anchor[2];

    if (dx * dx + dy * dy + dz * dz > RATE_EST_START_ACCEL_CHANGE * RATE_EST_START_ACCEL_CHANGE)
    {
        ResetCollection();
        return;
    }

    StatsAdd(&s_startup.gyro, gyro->x, gyro->y, gyro->z);

    if (!s_startup.have_accel || accel->sequence != s_startup.accel_sequence)
    {
        StatsAdd(&s_startup.accel, accel->x, accel->y, accel->z);
        s_startup.accel_sequence = accel->sequence;
        s_startup.have_accel = true;
    }

    if (!s_startup.have_baro || baro->sequence != s_startup.baro_sequence)
    {
        s_startup.pressure_count++;
        s_startup.pressure_mean += (baro->data.pressure_pa - s_startup.pressure_mean) / (float)s_startup.pressure_count;
        s_startup.baro_sequence = baro->sequence;
        s_startup.have_baro = true;
    }

    if (s_port_config.mag_calibrated && (!s_startup.have_mag || mag->sequence != s_startup.mag_sequence))
    {
        StatsAdd(&s_startup.mag, mag->data.x_uT, mag->data.y_uT, mag->data.z_uT);
        s_startup.mag_sequence = mag->sequence;
        s_startup.have_mag = true;
    }

    if ((uint32_t)(now - s_startup.start_tick) < MillisecondsToTicks(RATE_EST_COLLECT_MS) ||
        s_startup.gyro.count < RATE_EST_MIN_GYRO_SAMPLES ||
        s_startup.accel.count < RATE_EST_MIN_ACCEL_SAMPLES ||
        s_startup.pressure_count < RATE_EST_MIN_BARO_SAMPLES ||
        (s_port_config.mag_calibrated && s_startup.mag.count < RATE_EST_MIN_MAG_SAMPLES))
    {
        return;
    }

    if (!StatsQuiet(&s_startup.gyro, RATE_EST_GYRO_STD_MAX) || !StatsQuiet(&s_startup.accel, RATE_EST_ACCEL_STD_MAX))
    {
        ResetCollection();
        return;
    }

    Estimator_Config_t config;
    const float *mag_mean = NULL;

    Estimator_DefaultConfig(&config);

    if (s_port_config.mag_calibrated)
    {
        mag_mean = s_startup.mag.mean;
    }

    if (!EstimatorPort_Init(&s_port, &config, &s_port_config, s_startup.accel.mean, s_startup.gyro.mean, mag_mean, s_startup.pressure_mean))
    {
        Fault(RATE_EST_FAULT_INIT);
        return;
    }

    /* Establish IMU processing baseline. */
    (void)EstimatorPort_ProcessImuSamples(&s_port, gyro, accel, RATE_EST_DT_S, false);

    SetState(RATE_EST_RUNNING);

    /* Mark startup aiding samples as consumed. */
    s_runtime.baro_sequence = baro->sequence;
    s_runtime.mag_sequence = mag->sequence;
    s_runtime.have_baro = true;
    s_runtime.have_mag = mag->valid != 0U;
}

/* Fuse each fresh aiding publication once. */
static void ProcessAiding(const BaroTask_Sample_t *baro, const CompassTask_Sample_t *mag)
{
    if (s_runtime.baro_fresh && (!s_runtime.have_baro || baro->sequence != s_runtime.baro_sequence))
    {
        s_runtime.baro_sequence = baro->sequence;
        s_runtime.have_baro = true;

        if (s_runtime.state == RATE_EST_RUNNING)
        {
            (void)EstimatorPort_ProcessBaro(&s_port, &baro->data);
        }
    }

    if (s_runtime.mag_fresh && (!s_runtime.have_mag || mag->sequence != s_runtime.mag_sequence))
    {
        s_runtime.mag_sequence = mag->sequence;
        s_runtime.have_mag = true;

        if (s_runtime.state == RATE_EST_RUNNING && s_port_config.mag_calibrated)
        {
            (void)EstimatorPort_ProcessMag(&s_port, &mag->data);
        }
    }

    if (s_runtime.state == RATE_EST_RUNNING && !s_port.filter.healthy)
    {
        Fault(RATE_EST_FAULT_NUMERICAL);
    }
}

/* Return true after a successful estimator prediction. */
static bool ProcessGyro(uint32_t now, const IMUTask_Sample_t *gyro, const IMUTask_Sample_t *accel, const BaroTask_Sample_t *baro, const CompassTask_Sample_t *mag)
{
    if (s_runtime.state == RATE_EST_FAULT)
    {
        return false;
    }

    if (s_runtime.state != RATE_EST_RUNNING)
    {
        CollectStartup(now, gyro, accel, baro, mag);
        return false;
    }

    const float wx = gyro->x - s_port.filter.bg[0];
    const float wy = gyro->y - s_port.filter.bg[1];
    const float wz = gyro->z - s_port.filter.bg[2];
    const bool low_dynamics = RATE_EST_BENCH_GRAVITY_AID != 0U &&
        wx * wx + wy * wy + wz * wz < 0.15f * 0.15f &&
        fabsf(SampleNorm(accel) - GRAVITY_MPS2) < 0.5f;

    if (!EstimatorPort_ProcessImuSamples(&s_port, gyro, accel, RATE_EST_DT_S, low_dynamics))
    {
        Fault(RATE_EST_FAULT_PREDICT);
        return false;
    }

    return true;
}

/* Refresh the controller output and apply validity gates. */
static void UpdateOutput(void)
{
    if (!s_port.filter.initialized)
    {
        return;
    }

    if (!Estimator_GetOutput(&s_port.filter, &s_output))
    {
        Fault(RATE_EST_FAULT_OUTPUT);
    }

    const bool usable = s_runtime.state == RATE_EST_RUNNING &&
        !s_runtime.gyro_stale && !s_port.needs_resync && s_output.numerical_ok;

    s_output.numerical_ok = s_output.numerical_ok && usable;
    s_output.tilt_aided = s_output.tilt_aided && usable;
    s_output.height_aided = s_output.height_aided && usable && s_runtime.baro_fresh;
    s_output.yaw_aided = s_output.yaw_aided && usable && s_runtime.mag_fresh;
}

/* Signal a published gyro sample from IMUTask context. */
void RateControlTask_NotifyGyroReady(void)
{
    if (RateControlTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(RateControlTaskHandle, RATE_FLAG_GYRO_READY);
    }
}

/* Run estimator, flight control and motor service. */
void RateControlTask_Run(void *argument)
{
    (void)argument;

    s_runtime.tick_freq = osKernelGetTickFreq();
    s_runtime.imu_timeout = MillisecondsToTicks(RATE_EST_IMU_TIMEOUT_MS);
    s_runtime.aid_timeout = MillisecondsToTicks(RATE_EST_AID_TIMEOUT_MS);
    s_runtime.boot_tick = osKernelGetTickCount();

    SetState(RATE_EST_WAIT_SENSORS);

    static ControlConfig_t control_config;
    ControlConfig_Default(&control_config);
    (void)FlightControl_Init(&control_config, s_runtime.tick_freq, s_runtime.boot_tick);

    const uint32_t service_ticks = MillisecondsToTicks(1U);

    for (;;)
    {
        const uint32_t flags = osThreadFlagsWait(RATE_FLAG_GYRO_READY, osFlagsWaitAny, service_ticks);
        const uint32_t now = osKernelGetTickCount();
        bool estimator_stepped = false;

        IMUTask_Sample_t gyro = {0};
        IMUTask_Sample_t accel = {0};
        BaroTask_Sample_t baro = {0};
        CompassTask_Sample_t mag = {0};

        /* Update sensor age before processing the current event. */
        CheckImuAges(now);
        ReadAiding(now, &baro, &mag);

        if ((flags & osFlagsError) != 0U)
        {
            if (flags != osFlagsErrorTimeout)
            {
                Fault(RATE_EST_FAULT_FLAGS);
                (void)osDelay(1U);
            }
        }
        else if ((flags & RATE_FLAG_GYRO_READY) != 0U)
        {
            const bool gyro_valid = IMUTask_GetGyroSample(&gyro) != 0U && ImuFinite(&gyro);
            const bool accel_valid = IMUTask_GetAccSample(&accel) != 0U && ImuFinite(&accel);

            if (!gyro_valid || !accel_valid)
            {
                s_runtime.gyro_stale = true;

                if (s_runtime.state == RATE_EST_RUNNING)
                {
                    Fault(RATE_EST_FAULT_IMU_INVALID);
                }
                else
                {
                    ResetCollection();
                }
            }
            else
            {
                if (!s_runtime.have_accel || accel.sequence != s_runtime.accel_sequence)
                {
                    s_runtime.have_accel = true;
                    s_runtime.accel_sequence = accel.sequence;
                    s_runtime.accel_tick = now;
                }

                const uint32_t delta = (uint32_t)(gyro.sequence - s_runtime.gyro_sequence);

                if (!s_runtime.have_gyro || delta != 0U)
                {
                    if (s_runtime.have_gyro && delta != 1U)
                    {
                        if (s_runtime.state == RATE_EST_RUNNING)
                        {
                            Fault(RATE_EST_FAULT_GYRO_GAP);
                        }
                        else
                        {
                            ResetCollection();
                        }
                    }

                    s_runtime.have_gyro = true;
                    s_runtime.gyro_sequence = gyro.sequence;
                    s_runtime.gyro_tick = now;

                    s_runtime.gyro_stale = false;

                    /* Use the latest accelerometer sample within its age limit. */
                    if ((uint32_t)(now - s_runtime.accel_tick) < s_runtime.imu_timeout)
                    {
                        estimator_stepped = ProcessGyro(now, &gyro, &accel, &baro, &mag);
                    }
                }
            }
        }

        ProcessAiding(&baro, &mag);

        const uint32_t finished_tick = osKernelGetTickCount();

        CheckImuAges(finished_tick);
        UpdateOutput();
        FlightControl_Poll(finished_tick, &s_output, s_runtime.state == RATE_EST_RUNNING, s_runtime.state == RATE_EST_FAULT);

        if (estimator_stepped && s_runtime.state == RATE_EST_RUNNING)
        {
            FlightControl_Step(finished_tick, &s_output, &accel);
        }

        /* Update IMU freshness before motor service. */
        const uint32_t motor_tick = osKernelGetTickCount();

        CheckImuAges(motor_tick);

        if (s_runtime.state == RATE_EST_FAULT)
        {
            FlightControl_Poll(motor_tick, &s_output, false, true);
        }

        FlightControl_Service(motor_tick);
    }
}
