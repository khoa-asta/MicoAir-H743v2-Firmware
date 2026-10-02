#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "control_config.h"
#include "estimator.h"
#include "imu_task.h"

/* Flight-control state. */
typedef enum
{
    FLIGHT_DISARMED = 0,
    FLIGHT_ARMED,
    FLIGHT_LOCKOUT,
    FLIGHT_FAULT
} FlightState_t;

/* Flight-control block reasons. */
enum
{
    CONTROL_BLOCK_CONFIG    = 1U,
    CONTROL_BLOCK_RC        = 2U,
    CONTROL_BLOCK_ESTIMATOR = 4U,
    CONTROL_BLOCK_THROTTLE  = 8U,
    CONTROL_BLOCK_ARM_EDGE  = 16U,
    CONTROL_BLOCK_KILL      = 32U,
    CONTROL_BLOCK_MOTOR     = 64U,
    CONTROL_BLOCK_MODE      = 128U,
    CONTROL_BLOCK_TILT      = 256U,
    CONTROL_BLOCK_HEIGHT    = 512U,
    CONTROL_BLOCK_MATH      = 1024U
};

/**
  * @brief  Initializes control state, filters, PID controllers and motor output.
  */
bool FlightControl_Init(const ControlConfig_t *config, uint32_t tick_frequency, uint32_t now);

/**
  * @brief  Updates RC input, arming qualification and flight state.
  */
void FlightControl_Poll(uint32_t now, const Estimator_Output_t *estimate, bool estimator_running, bool estimator_fault);

/**
  * @brief  Runs scheduled attitude, vertical and body-rate control loops.
  */
void FlightControl_Step(uint32_t now, const Estimator_Output_t *estimate, const IMUTask_Sample_t *accel);

/**
  * @brief  Services motor transport and handles output faults.
  */
void FlightControl_Service(uint32_t now);

#endif
