#ifndef CONTROL_MATH_H
#define CONTROL_MATH_H
#include <stdbool.h>
#include <stdint.h>
#include "quaternion.h"

typedef struct { float b0,b1,b2,a1,a2,z1,z2; bool seeded; } ControlLpf_t;
typedef struct { float kp,ki,kd,kff,i_limit,out_limit,aw_gain; } ControlPidConfig_t;
typedef struct {
    ControlPidConfig_t config;
    float integral, requested, p_term, d_term;
    bool have_request;
} ControlPid_t;
typedef struct {
    float motor[4], achieved[3], collective, rp_scale, yaw_scale;
    bool saturated;
} QuadMixerOutput_t;

float Control_Clamp(float value,float low,float high);
bool ControlLpf_Init(ControlLpf_t *f,float cutoff_hz,float sample_hz);
bool ControlNotch_Init(ControlLpf_t *f,float center_hz,float bandwidth_hz,float sample_hz);
void ControlLpf_Reset(ControlLpf_t *f,float value);
bool ControlLpf_Update(ControlLpf_t *f,float input,float *output);
bool ControlPid_Init(ControlPid_t *p,const ControlPidConfig_t *config);
void ControlPid_Reset(ControlPid_t *p);
/* derivative is d(measurement)/dt; applied is previous achieved actuator effort. */
bool ControlPid_Update(ControlPid_t *p,float setpoint,float measurement,float derivative,
                       float applied,float dt,bool integrate,float *output);
/* Order: front-right, rear-left, front-left, rear-right. See geometry table. */
bool QuadMixer_Allocate(float collective,const float effort[3],float low,float high,QuadMixerOutput_t *out);
void QuadMixer_Inverse(const float motor[4],float *collective,float effort[3]);
/* Full quaternion P; body error, shortest path. Not PX4 reduced-attitude control. */
bool AttitudeControl_Update(const Quaternion_t *measured,const Quaternion_t *desired,
                            const float kp[3],const float limit[3],float yaw_rate_nav,float rate_sp[3]);
#endif
