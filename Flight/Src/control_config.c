#include "control_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/**
  * @brief  Loads default flight-control parameters.
  */
void ControlConfig_Default(ControlConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));

    /* CRSF channel mapping. */
    const uint8_t channel[CONTROL_RC_FUNCTIONS] = {0, 1, 2, 3, 5, 6, 7};
    memcpy(config->rc_channel, channel, sizeof(channel));

    /* RC channel limits. */
    for (int i = 0; i < CONTROL_RC_FUNCTIONS; i++)
    {
        config->rc_min[i] = 172;
        config->rc_center[i] = 992;
        config->rc_max[i] = 1811;
    }

    /* Control-axis directions. */
    config->axis_sign[0] = 1;
    config->axis_sign[1] = -1;
    config->axis_sign[2] = 1;
    config->axis_sign[3] = 1;
    config->yaw_motor_sign = 1;

    /* Configuration verification flags. */
    config->rc_verified = CONTROL_RC_VERIFIED != 0;
    config->geometry_verified = CONTROL_GEOMETRY_VERIFIED != 0;

    /* RC switch polarity. */
    config->arm_active_high = true;
    config->kill_active_high = true;
    config->altitude_enabled = CONTROL_ALTITUDE_ENABLED != 0;

    /* Flight-mode switch slots. */
    config->mode_slot[0] = CONTROL_RATE;
    config->mode_slot[1] = CONTROL_ATTITUDE;
    config->mode_slot[2] = CONTROL_ALTITUDE;

    /* Logical motor mapping. */
    for (int i = 0; i < 4; i++)
    {
        config->logical_motor[i] = (uint8_t)i;
    }

    /* Manual-control shaping. */
    config->deadzone = 0.1f;
    config->expo = 0.6f;
    config->max_tilt_rad = 0.523598776f;
    config->manual_yaw_rate = 2.617993878f;
    config->yaw_command_tau = 0.08f;

    /* Angular-rate limits. */
    config->rate_limit[0] = 3.839724354f;
    config->rate_limit[1] = 3.839724354f;
    config->rate_limit[2] = 6.283185307f;

    /* Attitude proportional gains. */
    config->attitude_kp[0] = 6.5f;
    config->attitude_kp[1] = 6.5f;
    config->attitude_kp[2] = 2.8f;

    /* Acro angular-rate limits. */
    for (int i = 0; i < 3; i++)
    {
        config->acro_rate[i] = 1.745329252f;
    }

    /* Sensor and derivative filters. */
    config->gyro_lpf_hz = 80;
    config->dterm_lpf_hz = 40;
    config->accel_lpf_hz = 30;
    config->notch_hz = 0;
    config->notch_bandwidth_hz = 20;

    /* Rate PID parameters. */
    config->rate_pid[0] = (ControlPidConfig_t)
    {
        .kp = 0.06f,
        .ki = 0.08f,
        .kd = 0.00128f,
        .i_limit = 0.3f,
        .out_limit = 1,
        .aw_gain = 5,
    };

    config->rate_pid[1] = (ControlPidConfig_t)
    {
        .kp = 0.06f,
        .ki = 0.08f,
        .kd = 0.00120f,
        .i_limit = 0.3f,
        .out_limit = 1,
        .aw_gain = 5,
    };

    config->rate_pid[2] = (ControlPidConfig_t)
    {
        .kp = 0.24f,
        .ki = 0.12f,
        .kd = 0,
        .i_limit = 0.3f,
        .out_limit = 1,
        .aw_gain = 5,
    };

    /* Vertical PID parameters. */
    config->vertical_pid = (ControlPidConfig_t)
    {
        .kp = 4,
        .ki = 2,
        .kd = 0,
        .i_limit = 3,
        .out_limit = 4,
        .aw_gain = 2,
    };

    /* Motor-output parameters. */
    config->motor_idle = 0.06f;
    config->motor_max = 1;
    config->thrust_curve = 0;

    /* Vertical-control parameters. */
    config->hover_thrust = 0.5f;
    config->height_p = 1;
    config->climb_up = 1;
    config->climb_down = 0.5f;
    config->vertical_accel_limit = 2;
    config->thrust_slew_per_s = 0.5f;

    /* Control timeouts. */
    config->rc_timeout_ms = 100;
    config->arm_low_ms = 500;
    config->motor_timeout_ms = 10;
}

/**
  * @brief  Validates channel mapping, controller parameters and safety timing.
  */
bool ControlConfig_Validate(const ControlConfig_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    if (config->yaw_motor_sign != 1 && config->yaw_motor_sign != -1)
    {
        return false;
    }

    /* Validate RC channel configuration. */
    for (int i = 0; i < CONTROL_RC_FUNCTIONS; i++)
    {
        if (config->rc_channel[i] >= 16 ||
            config->rc_max[i] > 2047 ||
            config->rc_min[i] >= config->rc_center[i] ||
            config->rc_center[i] >= config->rc_max[i])
        {
            return false;
        }
        for (int j = 0; j < i; j++)
        {
            if (config->rc_channel[i] == config->rc_channel[j])
            {
                return false;
            }
        }
    }

    /* Validate motor mapping and axis signs. */
    unsigned mask = 0;
    for (int i = 0; i < 4; i++)
    {
        if (config->logical_motor[i] >= 4 ||
            (mask & (1U << config->logical_motor[i])) ||
            (config->axis_sign[i] != 1 && config->axis_sign[i] != -1))
        {
            return false;
        }
        mask |= 1U << config->logical_motor[i];
    }

    /* Validate normalized parameters. */
    const float nonnegative[] =
    {
        config->deadzone,
        config->expo,
        config->thrust_curve,
        config->yaw_command_tau,
    };
    for (unsigned i = 0; i < sizeof(nonnegative) / sizeof(nonnegative[0]); i++)
    {
        if (!isfinite(nonnegative[i]) || nonnegative[i] < 0 || nonnegative[i] > 1)
        {
            return false;
        }
    }

    if (config->deadzone >= 0.5f)
    {
        return false;
    }

    /* Validate positive control parameters. */
    const float positive[] =
    {
        config->max_tilt_rad,
        config->manual_yaw_rate,
        config->motor_idle,
        config->motor_max,
        config->hover_thrust,
        config->height_p,
        config->climb_up,
        config->climb_down,
        config->vertical_accel_limit,
        config->thrust_slew_per_s,
    };
    for (unsigned i = 0; i < sizeof(positive) / sizeof(positive[0]); i++)
    {
        if (!isfinite(positive[i]) || positive[i] <= 0)
        {
            return false;
        }
    }

    if (config->max_tilt_rad > 0.785398164f ||
        config->motor_idle >= config->motor_max ||
        config->motor_max > 1 ||
        config->hover_thrust <= config->motor_idle ||
        config->hover_thrust >= config->motor_max ||
        config->rc_timeout_ms == 0 ||
        config->rc_timeout_ms > 1000 ||
        config->arm_low_ms == 0 ||
        config->motor_timeout_ms < 3 ||
        config->motor_timeout_ms > 100)
    {
        return false;
    }

    /* Validate controller parameters. */
    for (int i = 0; i < 3; i++)
    {
        ControlPid_t temp;
        if ((int)config->mode_slot[i] < 0 ||
            config->mode_slot[i] > CONTROL_POSITION ||
            !isfinite(config->acro_rate[i]) ||
            config->acro_rate[i] <= 0 ||
            config->acro_rate[i] > config->rate_limit[i] ||
            !isfinite(config->rate_limit[i]) ||
            config->rate_limit[i] <= 0 ||
            config->rate_limit[i] > 20 ||
            !isfinite(config->attitude_kp[i]) ||
            config->attitude_kp[i] <= 0 ||
            !ControlPid_Init(&temp, &config->rate_pid[i]))
        {
            return false;
        }
    }

    /* Validate filter parameters. */
    ControlPid_t temp;
    ControlLpf_t filter;
    if (!ControlLpf_Init(&filter, config->gyro_lpf_hz, 1000) ||
        !ControlLpf_Init(&filter, config->dterm_lpf_hz, 1000) ||
        !ControlLpf_Init(&filter, config->accel_lpf_hz, 800) ||
        !isfinite(config->notch_hz) ||
        config->notch_hz < 0 ||
        config->notch_hz >= 490 ||
        !isfinite(config->notch_bandwidth_hz) ||
        config->notch_bandwidth_hz <= 0 ||
        (config->notch_hz > 0 && config->notch_bandwidth_hz >= config->notch_hz))
    {
        return false;
    }

    return ControlPid_Init(&temp, &config->vertical_pid);
}
