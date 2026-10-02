#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

#include "control_config.h"

/**
  * @brief  Initializes motor commands and DShot transport state.
  */
bool MotorOutput_Init(const ControlConfig_t *config, uint32_t tick_frequency, uint32_t now);

/**
  * @brief  Stores validated thrust commands or requests motor stop.
  */
bool MotorOutput_SetDesired(const float motor[4], bool enabled, uint32_t now);

/**
  * @brief  Supervises output timeouts and advances motor transport.
  */
void MotorOutput_Service(uint32_t now);

/**
  * @brief  Reports initialized motor output without a latched fault.
  */
bool MotorOutput_Healthy(void);

/**
  * @brief  Reports motor readiness after a completed stop frame.
  */
bool MotorOutput_ReadyToArm(void);

/**
  * @brief  Copies thrust values from the latest completed motor frame.
  */
void MotorOutput_GetApplied(float motor[4]);

#endif
