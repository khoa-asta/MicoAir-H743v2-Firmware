#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include "control_math.h"

/* Control feature switches. */
#ifndef CONTROL_ENABLE_DSHOT
#define CONTROL_ENABLE_DSHOT 1
#endif

#ifndef CONTROL_RC_VERIFIED
#define CONTROL_RC_VERIFIED 0
#endif

#ifndef CONTROL_GEOMETRY_VERIFIED
#define CONTROL_GEOMETRY_VERIFIED 0
#endif

#ifndef CONTROL_ALTITUDE_ENABLED
#define CONTROL_ALTITUDE_ENABLED 0
#endif

/* Flight control modes. */
typedef enum
{
    CONTROL_RATE = 0,
    CONTROL_ATTITUDE = 1,
    CONTROL_ALTITUDE = 2,
    CONTROL_POSITION = 3
} ControlMode_t;

/* RC control functions. */
enum
{
    RC_ROLL = 0,
    RC_PITCH,
    RC_THROTTLE,
    RC_YAW,
    RC_ARM,
    RC_KILL,
    RC_MODE,
    CONTROL_RC_FUNCTIONS
};

/* Flight control configuration. */
typedef struct
{
    /* Control validation states. */
    bool rc_verified;
    bool geometry_verified;
    bool altitude_enabled;

    /* RC switch polarity. */
    bool arm_active_high;
    bool kill_active_high;

    /* RC channel mapping and calibration. */
    uint8_t rc_channel[CONTROL_RC_FUNCTIONS];
    uint16_t rc_min[CONTROL_RC_FUNCTIONS];
    uint16_t rc_center[CONTROL_RC_FUNCTIONS];
    uint16_t rc_max[CONTROL_RC_FUNCTIONS];

    /* Axis signs: roll right+, pitch nose-up+, throttle up+, yaw right+. */
    int8_t axis_sign[4];

    /* Yaw motor sign: +1 = FR/RL CCW and FL/RR CW from above. */
    int8_t yaw_motor_sign;

    /* Flight mode switch mapping. */
    ControlMode_t mode_slot[3];

    /* Geometry FR/RL/FL/RR to zero-based DShot M1..M4 mapping. */
    uint8_t logical_motor[4];

    /* Manual control shaping. */
    float deadzone;
    float expo;
    float max_tilt_rad;
    float manual_yaw_rate;
    float yaw_command_tau;

    /* Rate and attitude control limits. */
    float rate_limit[3];
    float attitude_kp[3];
    float acro_rate[3];

    /* Sensor and controller filters. */
    float gyro_lpf_hz;
    float dterm_lpf_hz;
    float accel_lpf_hz;
    float notch_hz;              /* Zero disables static notch. */
    float notch_bandwidth_hz;

    /* PID configuration. */
    ControlPidConfig_t rate_pid[3];
    ControlPidConfig_t vertical_pid;

    /* Motor output configuration. */
    float motor_idle;
    float motor_max;
    float thrust_curve;
    float thrust_slew_per_s;

    /* Vertical control configuration. */
    float hover_thrust;
    float height_p;
    float climb_up;
    float climb_down;
    float vertical_accel_limit;

    /* Safety timing. */
    uint32_t rc_timeout_ms;
    uint32_t arm_low_ms;
    uint32_t motor_timeout_ms;
} ControlConfig_t;

/* Load default control configuration. */
void ControlConfig_Default(ControlConfig_t *config);

/* Validate control configuration. */
bool ControlConfig_Validate(const ControlConfig_t *config);

#endif
