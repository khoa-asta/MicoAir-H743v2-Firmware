#include "flight_control.h"
#include "manual_control.h"
#include "motor_output.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RATE_DT 0.001f
#define ATT_DT 0.004f
#define VERTICAL_DT 0.020f
#define GRAVITY 9.80665f

static struct
{
    /* Configuration and RC commands. */
    ControlConfig_t config;
    ManualControl_t manual;

    /* Controller and filter states. */
    ControlPid_t rate_pid[3];
    ControlPid_t vertical_pid;
    ControlLpf_t notch[3];
    ControlLpf_t gyro_lpf[3];
    ControlLpf_t d_lpf[3];
    ControlLpf_t accel_lpf[3];

    /* Flight mode and arming state. */
    FlightState_t state;
    ControlMode_t mode;
    Quaternion_t desired;
    uint32_t tick_freq;
    uint32_t low_since;
    uint32_t accel_sequence;
    uint8_t att_div;
    uint8_t outer_div;
    bool initialized;
    bool last_arm;
    bool low_tracking;
    bool arm_qualified;
    bool filter_seeded;
    bool have_accel;
    bool mode_pending;
    bool vertical_active;

    /* Body rates in rad/s and derivatives in rad/s2. */
    float rate[3];
    float previous_rate[3];
    float derivative[3];
    /* Filtered body acceleration in m/s2. */
    float accel_b[3];
    float rate_sp[3];

    /* Attitude, height and thrust setpoints. */
    float yaw_sp;
    float yaw_rate;
    float height_sp;
    float climb_sp;
    float thrust_target;
    float collective;
    float accel_up;
} s_control;

/**
  * @brief  Update flight state.
  */
static void SetState(FlightState_t state)
{
    s_control.state = state;
}

/**
  * @brief  Extract yaw angle from body-to-navigation quaternion.
  */
static float Yaw(const Quaternion_t *q)
{
    return atan2f(2 * (q->w * q->z + q->x * q->y), 1 - 2 * (q->y * q->y + q->z * q->z));
}

/**
  * @brief  Wrap an angle to the [-pi, pi] range.
  */
static float WrapPi(float x)
{
    return remainderf(x, 6.28318530718f);
}

/**
  * @brief  Reset controller states and setpoints.
  */
static void ResetControl(const Estimator_Output_t *estimate)
{
    for (int i = 0; i < 3; i++)
    {
        ControlPid_Reset(&s_control.rate_pid[i]);
        s_control.rate_sp[i] = 0;
    }

    ControlPid_Reset(&s_control.vertical_pid);

    s_control.att_div = 0;
    s_control.outer_div = 0;
    s_control.yaw_rate = 0;
    s_control.vertical_active = false;
    s_control.collective = s_control.config.motor_idle;
    s_control.thrust_target = s_control.config.motor_idle;
    s_control.climb_sp = 0;

    if (estimate != NULL)
    {
        s_control.height_sp = estimate->height_m;
        s_control.yaw_sp = Yaw(&estimate->q_nb);
    }
    else
    {
        s_control.height_sp = 0;
        s_control.yaw_sp = 0;
    }

    s_control.mode_pending = true;
}

/**
  * @brief  Stop control output and update flight state.
  */
static void Stop(uint32_t reason, bool fatal, uint32_t now)
{
    if (fatal || s_control.state == FLIGHT_FAULT)
    {
        SetState(FLIGHT_FAULT);
    }
    else if (reason == 0)
    {
        SetState(FLIGHT_DISARMED);
    }
    else
    {
        SetState(FLIGHT_LOCKOUT);
    }

    s_control.arm_qualified = false;
    s_control.low_tracking = false;
    ResetControl(NULL);

    (void)MotorOutput_SetDesired(NULL, false, now);
}

/**
  * @brief  Initializes control state, filters, PID controllers and motor output.
  */
bool FlightControl_Init(const ControlConfig_t *config, uint32_t frequency, uint32_t now)
{
    memset(&s_control, 0, sizeof(s_control));

    if (!ControlConfig_Validate(config) || frequency == 0)
    {
        SetState(FLIGHT_FAULT);
        return false;
    }

    s_control.config = *config;
    s_control.tick_freq = frequency;
    s_control.mode = CONTROL_RATE;

    ManualControl_Init(&s_control.manual);

    /* Initialize rate-control filters and PID controllers. */
    for (int i = 0; i < 3; i++)
    {
        if (!ControlPid_Init(&s_control.rate_pid[i], &config->rate_pid[i]) ||
            !ControlLpf_Init(&s_control.gyro_lpf[i], config->gyro_lpf_hz, 1000) ||
            !ControlLpf_Init(&s_control.d_lpf[i], config->dterm_lpf_hz, 1000) ||
            !ControlLpf_Init(&s_control.accel_lpf[i], config->accel_lpf_hz, 800) ||
            !ControlNotch_Init(&s_control.notch[i], config->notch_hz, config->notch_bandwidth_hz, 1000))
        {
            SetState(FLIGHT_FAULT);
            return false;
        }
    }

    (void)ControlPid_Init(&s_control.vertical_pid, &config->vertical_pid);
    s_control.initialized = true;

    if (!MotorOutput_Init(&s_control.config, frequency, now))
    {
        Stop(CONTROL_BLOCK_MOTOR, true, now);
        return false;
    }

    SetState(FLIGHT_DISARMED);
    return true;
}

/**
  * @brief  Validate estimator output used by the controller.
  */
static bool EstimateIsValid(const Estimator_Output_t *estimate)
{
    Quaternion_t q;

    if (estimate == NULL ||
        !estimate->numerical_ok ||
        !Quaternion_Normalize(&estimate->q_nb, &q) ||
        fabsf(Quaternion_Norm(&estimate->q_nb) - 1.0f) > 0.02f ||
        !isfinite(estimate->height_m) ||
        !isfinite(estimate->climb_rate_mps))
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (!isfinite(estimate->body_rate_rad_s[i]) || !isfinite(estimate->accel_bias_mps2[i]))
        {
            return false;
        }
    }

    return true;
}

/**
  * @brief  Updates RC input, arming qualification and flight state.
  */
void FlightControl_Poll(uint32_t now, const Estimator_Output_t *estimate, bool running, bool estimator_fault)
{
    if (!s_control.initialized)
    {
        return;
    }

    ManualControl_Read(&s_control.manual, &s_control.config);

    const bool estimate_ok = running && EstimateIsValid(estimate);
    const bool rising = s_control.manual.valid && s_control.manual.arm && !s_control.last_arm;
    uint32_t block = 0;

    /* Evaluate flight-control block conditions. */
    if (!s_control.config.rc_verified || !s_control.config.geometry_verified)
    {
        block |= CONTROL_BLOCK_CONFIG;
    }
    if (!s_control.manual.valid)
    {
        block |= CONTROL_BLOCK_RC;
    }
    if (!estimate_ok)
    {
        block |= CONTROL_BLOCK_ESTIMATOR;
    }
    if (s_control.manual.kill)
    {
        block |= CONTROL_BLOCK_KILL;
    }
    if (!MotorOutput_Healthy())
    {
        block |= CONTROL_BLOCK_MOTOR;
    }
    if (s_control.manual.mode == CONTROL_POSITION ||
        (s_control.manual.mode == CONTROL_ALTITUDE && !s_control.config.altitude_enabled))
    {
        block |= CONTROL_BLOCK_MODE;
    }
    if (s_control.manual.mode == CONTROL_ALTITUDE && (!estimate_ok || !estimate->height_aided))
    {
        block |= CONTROL_BLOCK_HEIGHT;
    }

    if (estimator_fault)
    {
        Stop(CONTROL_BLOCK_ESTIMATOR, true, now);
    }
    if (!MotorOutput_Healthy())
    {
        Stop(CONTROL_BLOCK_MOTOR, true, now);
    }

    /* Handle armed-state transitions. */
    if (s_control.state == FLIGHT_ARMED)
    {
        if (block != 0)
        {
            Stop(block, false, now);
        }
        else if (!s_control.manual.arm)
        {
            Stop(0, false, now);
        }
        else if (s_control.mode != s_control.manual.mode)
        {
            const float collective = s_control.collective;
            ResetControl(estimate);
            s_control.collective = collective;
            s_control.thrust_target = collective;
            s_control.mode = s_control.manual.mode;
            s_control.vertical_active = s_control.mode == CONTROL_ALTITUDE &&
                                        collective > s_control.config.motor_idle + 0.02f;
        }
    }

    /* Handle disarmed-state arming qualification. */
    if (s_control.state != FLIGHT_ARMED && s_control.state != FLIGHT_FAULT)
    {
        if (s_control.manual.throttle > 0.05f)
        {
            block |= CONTROL_BLOCK_THROTTLE;
        }
        if (estimate_ok)
        {
            const float tilt = 1 - 2 * (estimate->q_nb.x * estimate->q_nb.x +
                                        estimate->q_nb.y * estimate->q_nb.y);

            if (!estimate->tilt_aided || tilt < 0.8660254f)
            {
                block |= CONTROL_BLOCK_TILT;
            }
        }
        if (!MotorOutput_ReadyToArm())
        {
            block |= CONTROL_BLOCK_MOTOR;
        }

        if (!s_control.manual.arm && block == 0)
        {
            if (!s_control.low_tracking)
            {
                s_control.low_tracking = true;
                s_control.low_since = now;
            }

            const uint32_t delay = (uint32_t)
                (((uint64_t)s_control.config.arm_low_ms * s_control.tick_freq + 999U) / 1000U);
            if ((uint32_t)(now - s_control.low_since) >= delay)
            {
                s_control.arm_qualified = true;
                SetState(FLIGHT_DISARMED);
            }
        }
        else if (block != 0)
        {
            s_control.low_tracking = false;
            s_control.arm_qualified = false;
        }
        else if (s_control.manual.arm && !s_control.arm_qualified)
        {
            s_control.low_tracking = false;
        }

        if (rising && s_control.arm_qualified && block == 0)
        {
            ResetControl(estimate);
            s_control.mode = s_control.manual.mode;
            SetState(FLIGHT_ARMED);
            s_control.arm_qualified = false;
            s_control.low_tracking = false;
        }
    }

    if (s_control.state != FLIGHT_ARMED)
    {
        (void)MotorOutput_SetDesired(NULL, false, now);
    }

    if (s_control.manual.valid)
    {
        s_control.last_arm = s_control.manual.arm;
    }
    else
    {
        s_control.last_arm = true;
    }
}

/**
  * @brief  Filters body rates, derivatives and accelerometer data.
  */
static bool FilterMeasurements(const Estimator_Output_t *estimate, const IMUTask_Sample_t *accel)
{
    for (int i = 0; i < 3; i++)
    {
        float notched;

        if (!ControlLpf_Update(&s_control.notch[i], estimate->body_rate_rad_s[i], &notched) ||
            !ControlLpf_Update(&s_control.gyro_lpf[i], notched, &s_control.rate[i]))
        {
            return false;
        }

        float derivative = 0;

        if (s_control.filter_seeded)
        {
            derivative = (s_control.rate[i] - s_control.previous_rate[i]) / RATE_DT;
        }

        if (!ControlLpf_Update(&s_control.d_lpf[i], derivative, &s_control.derivative[i]))
        {
            return false;
        }

        s_control.previous_rate[i] = s_control.rate[i];
    }

    s_control.filter_seeded = true;

    /* Update filtered accelerometer sample. */
    if (!s_control.have_accel || accel->sequence != s_control.accel_sequence)
    {
        const float value[3] = {accel->x, accel->y, accel->z};
        for (int i = 0; i < 3; i++)
        {
            if (!ControlLpf_Update(&s_control.accel_lpf[i], value[i] - estimate->accel_bias_mps2[i], &s_control.accel_b[i]))
            {
                return false;
            }
        }

        s_control.have_accel = true;
        s_control.accel_sequence = accel->sequence;
    }

    float nav[3];
    if (!Quaternion_RotateVector(&estimate->q_nb, s_control.accel_b, nav))
    {
        return false;
    }

    s_control.accel_up = -nav[2] - GRAVITY;
    return isfinite(s_control.accel_up);
}

/**
  * @brief  Generate body-rate setpoint from attitude command.
  */
static bool UpdateAttitudeControl(const Estimator_Output_t *estimate)
{
    const float alpha = ATT_DT / (s_control.config.yaw_command_tau + ATT_DT);
    s_control.yaw_rate += alpha * (s_control.manual.yaw * s_control.config.manual_yaw_rate - s_control.yaw_rate);
    s_control.yaw_sp = WrapPi(s_control.yaw_sp + s_control.yaw_rate * ATT_DT);

    const float yaw = Yaw(&estimate->q_nb);
    const float yaw_error = WrapPi(s_control.yaw_sp - yaw);
    s_control.yaw_sp = WrapPi(yaw + Control_Clamp(yaw_error, -0.7853982f, 0.7853982f));

    const float norm = hypotf(s_control.manual.roll, s_control.manual.pitch);
    const float scale = s_control.config.max_tilt_rad / fmaxf(1, norm);

    if (!Quaternion_FromEulerZYX(scale * s_control.manual.roll,
                                 scale * s_control.manual.pitch,
                                 s_control.yaw_sp,
                                 &s_control.desired))
    {
        return false;
    }
    return AttitudeControl_Update(&estimate->q_nb,
                                  &s_control.desired,
                                  s_control.config.attitude_kp,
                                  s_control.config.rate_limit,
                                  s_control.yaw_rate,
                                  s_control.rate_sp);
}

/**
  * @brief  Generate collective-thrust target from vertical control.
  */
static bool UpdateVerticalControl(const Estimator_Output_t *estimate, float applied_collective)
{
    const float up = s_control.manual.up;

    if (fabsf(up) > 0.001f)
    {
        if (up > 0)
        {
            s_control.climb_sp = up * s_control.config.climb_up;
        }
        else
        {
            s_control.climb_sp = up * s_control.config.climb_down;
        }

        s_control.height_sp = estimate->height_m;
    }
    else
    {
        const float height_error = s_control.height_sp - estimate->height_m;
        s_control.climb_sp = Control_Clamp(s_control.config.height_p * height_error,
                                          -s_control.config.climb_down,
                                          s_control.config.climb_up);
    }

    const float tilt = 1 - 2 * (estimate->q_nb.x * estimate->q_nb.x + estimate->q_nb.y * estimate->q_nb.y);
    if (tilt < 0.5f)
    {
        return false;
    }

    const float achieved_accel = GRAVITY * (applied_collective * tilt / s_control.config.hover_thrust - 1);
    float accel_sp;

    if (!ControlPid_Update(&s_control.vertical_pid,
                           s_control.climb_sp,
                           estimate->climb_rate_mps,
                           s_control.accel_up,
                           achieved_accel,
                           VERTICAL_DT,
                           true,
                           &accel_sp))
    {
        return false;
    }

    accel_sp = Control_Clamp(accel_sp, -s_control.config.vertical_accel_limit, s_control.config.vertical_accel_limit);
    const float thrust = s_control.config.hover_thrust * (1 + accel_sp / GRAVITY) / tilt;
    s_control.thrust_target = Control_Clamp(thrust,
                                          s_control.config.motor_idle,
                                          s_control.config.motor_max);
    return true;
}

/**
  * @brief  Generate control effort from body-rate errors.
  */
static bool UpdateRateControl(const float applied_effort[3], float effort[3])
{
    const bool integrate = s_control.collective > s_control.config.motor_idle + 0.02f;

    for (int i = 0; i < 3; i++)
    {
        if (!ControlPid_Update(&s_control.rate_pid[i],
                               s_control.rate_sp[i],
                               s_control.rate[i],
                               s_control.derivative[i],
                               applied_effort[i],
                               RATE_DT,
                               integrate,
                               &effort[i]))
        {
            return false;
        }
    }

    return true;
}

/**
  * @brief  Runs scheduled attitude, vertical and body-rate control loops.
  */
void FlightControl_Step(uint32_t now, const Estimator_Output_t *estimate, const IMUTask_Sample_t *accel)
{
    if (!s_control.initialized || s_control.state == FLIGHT_FAULT)
    {
        return;
    }

    if (!EstimateIsValid(estimate) ||
        accel == NULL ||
        !accel->valid ||
        !FilterMeasurements(estimate, accel))
    {
        Stop(CONTROL_BLOCK_MATH, true, now);
        return;
    }

    if (s_control.state != FLIGHT_ARMED)
    {
        return;
    }

    if (s_control.mode == CONTROL_POSITION)
    {
        Stop(CONTROL_BLOCK_MODE, false, now);
        return;
    }

    /* Recover applied motor state. */
    float applied_motor[4];
    float applied_collective;
    float applied_effort[3];
    MotorOutput_GetApplied(applied_motor);
    QuadMixer_Inverse(applied_motor, &applied_collective, applied_effort);
    applied_effort[2] *= (float)s_control.config.yaw_motor_sign;

    /* Schedule outer control loops. */
    const bool first = s_control.mode_pending;
    const bool attitude_due = (++s_control.att_div >= 4U) || first;
    const bool vertical_due = (++s_control.outer_div >= 20U) || first;

    if (attitude_due)
    {
        s_control.att_div = 0;
    }
    if (vertical_due)
    {
        s_control.outer_div = 0;
    }
    s_control.mode_pending = false;

    if (s_control.mode == CONTROL_ALTITUDE && !s_control.vertical_active && s_control.manual.throttle > 0.55f)
    {
        s_control.vertical_active = true;
        s_control.height_sp = estimate->height_m;
        ControlPid_Reset(&s_control.vertical_pid);
    }

    bool idle;

    if (s_control.mode == CONTROL_ALTITUDE)
    {
        idle = !s_control.vertical_active;
    }
    else
    {
        idle = s_control.manual.throttle <= 0.05f;
    }

    /* Hold motor idle and reset controller state. */
    if (idle)
    {
        s_control.yaw_sp = Yaw(&estimate->q_nb);
        s_control.yaw_rate = 0;
        s_control.height_sp = estimate->height_m;
        s_control.collective = s_control.config.motor_idle;
        s_control.thrust_target = s_control.config.motor_idle;
        ControlPid_Reset(&s_control.vertical_pid);

        float idle_motor[4];
        for (int i = 0; i < 4; i++)
        {
            idle_motor[i] = s_control.config.motor_idle;
        }

        for (int i = 0; i < 3; i++)
        {
            ControlPid_Reset(&s_control.rate_pid[i]);
        }

        (void)MotorOutput_SetDesired(idle_motor, true, now);
        return;
    }

    /* Generate body-rate setpoints. */
    if (s_control.mode == CONTROL_RATE)
    {
        s_control.rate_sp[0] = s_control.manual.roll * s_control.config.acro_rate[0];
        s_control.rate_sp[1] = s_control.manual.pitch * s_control.config.acro_rate[1];
        s_control.rate_sp[2] = s_control.manual.yaw * s_control.config.acro_rate[2];
    }
    else if (attitude_due)
    {
        if (!UpdateAttitudeControl(estimate))
        {
            Stop(CONTROL_BLOCK_MATH, true, now);
            return;
        }
    }

    /* Update vertical-control target. */
    if (s_control.mode == CONTROL_ALTITUDE)
    {
        if (!estimate->height_aided)
        {
            Stop(CONTROL_BLOCK_HEIGHT, false, now);
            return;
        }

        if (vertical_due)
        {
            if (!UpdateVerticalControl(estimate, applied_collective))
            {
                Stop(CONTROL_BLOCK_MATH, true, now);
                return;
            }
        }
    }
    else
    {
        const float thrust_range = s_control.config.motor_max - s_control.config.motor_idle;
        s_control.thrust_target = s_control.config.motor_idle + s_control.manual.throttle * thrust_range;
    }

    /* Slew collective thrust. */
    const float delta = s_control.config.thrust_slew_per_s * RATE_DT;
    s_control.collective += Control_Clamp(s_control.thrust_target - s_control.collective, -delta, delta);

    /* Run body-rate PID controllers. */
    float effort[3];

    if (!UpdateRateControl(applied_effort, effort))
    {
        Stop(CONTROL_BLOCK_MATH, true, now);
        return;
    }

    /* Allocate control effort to four motors. */
    QuadMixerOutput_t mixed;
    effort[2] *= (float)s_control.config.yaw_motor_sign;

    if (!QuadMixer_Allocate(s_control.collective, effort, s_control.config.motor_idle, s_control.config.motor_max, &mixed))
    {
        Stop(CONTROL_BLOCK_MATH, true, now);
        return;
    }

    if (!MotorOutput_SetDesired(mixed.motor, true, now))
    {
        Stop(CONTROL_BLOCK_MOTOR, true, now);
        return;
    }
}

/**
  * @brief  Services motor transport and handles output faults.
  */
void FlightControl_Service(uint32_t now)
{
    if (!s_control.initialized)
    {
        return;
    }

    MotorOutput_Service(now);
    if (!MotorOutput_Healthy())
    {
        Stop(CONTROL_BLOCK_MOTOR, true, now);
    }
}
