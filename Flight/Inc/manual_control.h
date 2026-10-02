#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include "control_config.h"

/* Manual control state. */
typedef struct
{
    /* Normalized stick commands. */
    float roll;
    float pitch;
    float throttle;
    float yaw;
    float up;

    /* RC state and switches. */
    bool valid;
    bool arm;
    bool kill;

    /* Selected flight mode. */
    ControlMode_t mode;

    /* RC frame information. */
    uint32_t sequence;
    uint32_t age_ms;

    /* Mode switch position. */
    uint8_t mode_position;
} ManualControl_t;

/* Initialize manual control state. */
void ManualControl_Init(ManualControl_t *manual);
/* Read and convert RC input */
void ManualControl_Read(ManualControl_t *manual,const ControlConfig_t *config);

#endif
