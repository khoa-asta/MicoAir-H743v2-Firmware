/*
 * dshot300.h
 *
 * Author: Viết Khoa
 */

#ifndef DEVICE_DSHOT300_DSHOT300_H_
#define DEVICE_DSHOT300_DSHOT300_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT300_MOTOR_COUNT       4U
#define DSHOT300_FRAME_BITS        16U
#define DSHOT300_TX_ROWS           18U
#define DSHOT300_BIT0_TICKS        300U
#define DSHOT300_BIT1_TICKS        600U
#define DSHOT300_TX_TIMEOUT_MS     5U
#define DSHOT300_QUIET_TICKS       1U

typedef enum
{
    DSHOT300_UNINITIALIZED = 0,
    DSHOT300_READY = 1,
    DSHOT300_TX = 2,
    DSHOT300_QUIET = 3,
    DSHOT300_FAULT = 4
} DSHOT300_State_t;

/* Validate hardware and initialize transport. */
HAL_StatusTypeDef DSHOT300_Init(void);

/* Send values M1 to M4: 0 stops; 48 to 2047 sets throttle. */
HAL_StatusTypeDef DSHOT300_Send(const uint16_t motor_values[DSHOT300_MOTOR_COUNT]);

/* Handle transfer timeout and quiet-interval completion. */
void DSHOT300_Process(void);

/* Report an active transfer or quiet interval. */
uint8_t DSHOT300_IsBusy(void);

/* Cancel transport without transmitting a stop frame. */
HAL_StatusTypeDef DSHOT300_Abort(void);

/* Return the current transport state. */
DSHOT300_State_t DSHOT300_GetState(void);

/* Return the sequence count of completed DMA transfers. */
uint32_t DSHOT300_GetCompletedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_DSHOT300_DSHOT300_H_ */
