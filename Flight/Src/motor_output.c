#include "motor_output.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#if CONTROL_ENABLE_DSHOT
#include "dshot300.h"
#endif

/* Motor commands and transport state. */
static const ControlConfig_t *s_config;
static float s_desired[4];
static float s_applied[4];
static uint32_t s_frequency;
static uint32_t s_last_progress;
static uint32_t s_desired_tick;
static bool s_enabled;
static bool s_healthy;
static bool s_initialized;

/* Completion of the latest requested stop frame. */
static bool s_stop_confirmed;

#if CONTROL_ENABLE_DSHOT
static float s_inflight[4];
static uint32_t s_complete_baseline;
static bool s_tracking;
static bool s_tracking_stop;
#endif

/**
  * @brief  Disables motor commands and requests a zero-throttle frame.
  */
static void StopRequest(void)
{
    s_enabled = false;
    s_stop_confirmed = false;
    memset(s_desired, 0, sizeof(s_desired));

#if CONTROL_ENABLE_DSHOT
    if (s_initialized && DSHOT300_GetState() == DSHOT300_TX)
    {
        s_tracking = false;

        if (DSHOT300_Abort() != HAL_OK)
        {
            s_healthy = false;
        }
    }
#endif
}

/**
  * @brief  Latches a motor-output fault and requests a stop frame.
  */
static void MotorFault(void)
{
    s_healthy = false;
    StopRequest();
}

/**
  * @brief  Initializes motor commands and DShot transport state.
  */
bool MotorOutput_Init(const ControlConfig_t *config, uint32_t frequency, uint32_t now)
{
    memset(s_desired, 0, sizeof(s_desired));
    memset(s_applied, 0, sizeof(s_applied));

    s_config = config;
    s_frequency = frequency;
    s_last_progress = now;
    s_desired_tick = now;
    s_enabled = false;
    s_healthy = true;
    s_initialized = false;
    s_stop_confirmed = false;

    if (!ControlConfig_Validate(config) || frequency == 0)
    {
        MotorFault();
        return false;
    }

#if CONTROL_ENABLE_DSHOT
    memset(s_inflight, 0, sizeof(s_inflight));
    s_tracking = false;
    s_tracking_stop = false;

    if (DSHOT300_Init() != HAL_OK)
    {
        MotorFault();
        return false;
    }

    s_complete_baseline = DSHOT300_GetCompletedCount();
#endif

    s_initialized = true;
    return true;
}

/**
  * @brief  Stores validated thrust commands or requests motor stop.
  */
bool MotorOutput_SetDesired(const float motor[4], bool enabled, uint32_t now)
{
    if (enabled)
    {
        if (!s_initialized || !s_healthy || motor == NULL)
        {
            MotorFault();
            return false;
        }

        for (int i = 0; i < 4; i++)
        {
            if (!isfinite(motor[i]) || motor[i] < 0 || motor[i] > 1)
            {
                MotorFault();
                return false;
            }
        }
    }

    if (!enabled)
    {
        if (s_enabled)
        {
            StopRequest();
        }
        else
        {
            memset(s_desired, 0, sizeof(s_desired));
        }
    }
    else
    {
        s_enabled = true;
        memcpy(s_desired, motor, sizeof(s_desired));
        s_desired_tick = now;
    }

    return s_healthy;
}

#if CONTROL_ENABLE_DSHOT
/**
  * @brief  Converts normalized thrust to a normalized DShot command.
  */
static float ThrustToCommand(float thrust)
{
    if (thrust <= 0)
    {
        return 0;
    }

    const float k = s_config->thrust_curve;

    if (k < 1.0e-6f)
    {
        return thrust;
    }

    const float a = 1 - k;
    return 2 * thrust / (sqrtf(a * a + 4 * k * thrust) + a);
}
#endif

/**
  * @brief  Supervises output timeouts and advances motor transport.
  */
void MotorOutput_Service(uint32_t now)
{
    if (!s_initialized)
    {
        return;
    }

    const uint32_t timeout = (uint32_t)
        (((uint64_t)s_config->motor_timeout_ms * s_frequency + 999U) / 1000U);

    /* Motor-command timeout. */
    if (s_enabled && (uint32_t)(now - s_desired_tick) >= timeout)
    {
        MotorFault();
    }

#if CONTROL_ENABLE_DSHOT
    DSHOT300_Process();

    if (DSHOT300_GetState() == DSHOT300_FAULT ||
        DSHOT300_GetState() == DSHOT300_UNINITIALIZED)
    {
        MotorFault();
        return;
    }

    /* Applied thrust and stop-frame completion. */
    if (s_tracking && DSHOT300_GetCompletedCount() != s_complete_baseline)
    {
        s_tracking = false;
        s_last_progress = now;
        memcpy(s_applied, s_inflight, sizeof(s_applied));

        if (s_tracking_stop)
        {
            s_stop_confirmed = true;
        }
    }

    if (s_healthy && (uint32_t)(now - s_last_progress) >= timeout)
    {
        MotorFault();
    }

    if (DSHOT300_GetState() != DSHOT300_READY)
    {
        return;
    }

    /* Logical motor mapping and DShot throttle values. */
    uint16_t values[4] = {0};

    if (s_enabled && s_healthy)
    {
        for (int i = 0; i < 4; i++)
        {
            const float command = Control_Clamp(ThrustToCommand(s_desired[i]), 0, 1);
            const uint16_t throttle = (uint16_t)lroundf(command * 1999.0f);
            values[s_config->logical_motor[i]] = (uint16_t)(48U + throttle);
        }
    }

    s_complete_baseline = DSHOT300_GetCompletedCount();
    const HAL_StatusTypeDef status = DSHOT300_Send(values);

    if (status == HAL_BUSY && DSHOT300_GetState() != DSHOT300_FAULT)
    {
        return;
    }

    if (status != HAL_OK)
    {
        MotorFault();
        return;
    }

    s_tracking = true;
    s_tracking_stop = !s_enabled;
    memcpy(s_inflight, s_desired, sizeof(s_inflight));
#else
    s_last_progress = now;
    memcpy(s_applied, s_desired, sizeof(s_applied));

    if (!s_enabled)
    {
        s_stop_confirmed = true;
    }
#endif
}

/**
  * @brief  Reports initialized motor output without a latched fault.
  */
bool MotorOutput_Healthy(void)
{
    return s_initialized && s_healthy;
}

/**
  * @brief  Reports motor readiness after a completed stop frame.
  */
bool MotorOutput_ReadyToArm(void)
{
    return MotorOutput_Healthy() && s_stop_confirmed;
}

/**
  * @brief  Copies thrust values from the latest completed motor frame.
  */
void MotorOutput_GetApplied(float motor[4])
{
    if (motor != NULL)
    {
        memcpy(motor, s_applied, sizeof(s_applied));
    }
}
