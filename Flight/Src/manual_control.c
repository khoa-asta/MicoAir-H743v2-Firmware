#include "manual_control.h"
#include "rc_task.h"
#include <math.h>
#include <string.h>

/**
  * @brief  Applies deadzone and expo to a normalized control axis.
  */
static float Shape(float x, const ControlConfig_t *config)
{
    const float a = fabsf(x);

    if (a <= config->deadzone)
    {
        return 0;
    }

    x = copysignf((a - config->deadzone) / (1 - config->deadzone), x);
    return (1 - config->expo) * x + config->expo * x * x * x;
}

/**
  * @brief  Initializes RC commands and switch state.
  */
void ManualControl_Init(ManualControl_t *manual)
{
    memset(manual, 0, sizeof(*manual));
    manual->kill = true;
    manual->age_ms = UINT32_MAX;
}

/**
  * @brief  Converts a valid RC snapshot to manual control commands.
  */
void ManualControl_Read(ManualControl_t *manual, const ControlConfig_t *config)
{
    RC_Snapshot_t rc = {0};
    manual->valid = false;

    /* Read the latest RC snapshot. */
    if (RCTask_GetSnapshot(&rc) == 0U ||
        !rc.has_frame ||
        rc.link_state != RC_LINK_ACTIVE ||
        rc.age_ms >= config->rc_timeout_ms)
    {
        manual->age_ms = rc.age_ms;
        return;
    }

    /* Normalize configured RC channels. */
    float normalized[CONTROL_RC_FUNCTIONS];
    for (int i = 0; i < CONTROL_RC_FUNCTIONS; i++)
    {
        const float value = rc.channels[config->rc_channel[i]];
        const float span = (float)(config->rc_max[i] - config->rc_min[i]);

        if (value > 2047 ||
            value < config->rc_min[i] - 0.15f * span ||
            value > config->rc_max[i] + 0.15f * span)
        {
            return;
        }
        normalized[i] = Control_Clamp((value - config->rc_min[i]) / span, 0, 1);
    }

    /* Shape roll, pitch and yaw axes. */
    const int axes[3] = {RC_ROLL, RC_PITCH, RC_YAW};
    float shaped[3];

    for (int j = 0; j < 3; j++)
    {
        const int i = axes[j];
        const float raw = rc.channels[config->rc_channel[i]];
        const float center = config->rc_center[i];
        float span;

        if (raw >= center)
        {
            span = config->rc_max[i] - center;
        }
        else
        {
            span = center - config->rc_min[i];
        }

        const float x = Control_Clamp((raw - center) / span, -1, 1) * config->axis_sign[i];
        shaped[j] = Shape(x, config);
    }

    manual->roll = shaped[0];
    manual->pitch = shaped[1];
    manual->yaw = shaped[2];

    /* Throttle and vertical command. */
    if (config->axis_sign[RC_THROTTLE] > 0)
    {
        manual->throttle = normalized[RC_THROTTLE];
    }
    else
    {
        manual->throttle = 1 - normalized[RC_THROTTLE];
    }

    manual->up = Shape(2 * manual->throttle - 1, config);

    /* Arm and kill switches. */
    float arm = normalized[RC_ARM];
    float kill = normalized[RC_KILL];

    if (!config->arm_active_high)
    {
        arm = 1 - arm;
    }

    if (!config->kill_active_high)
    {
        kill = 1 - kill;
    }

    if (arm > 0.7f)
    {
        manual->arm = true;
    }
    else if (arm < 0.3f)
    {
        manual->arm = false;
    }

    if (kill > 0.7f)
    {
        manual->kill = true;
    }
    else if (kill < 0.3f)
    {
        manual->kill = false;
    }

    /* Three-position flight-mode switch. */
    const float mode = normalized[RC_MODE];

    if (mode < 0.28f)
    {
        manual->mode_position = 0;
    }
    else if (mode > 0.72f)
    {
        manual->mode_position = 2;
    }
    else if (mode > 0.38f && mode < 0.62f)
    {
        manual->mode_position = 1;
    }

    manual->mode = config->mode_slot[manual->mode_position];
    manual->sequence = rc.sequence;
    manual->age_ms = rc.age_ms;
    manual->valid = true;
}
