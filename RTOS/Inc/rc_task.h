/*
 * rc_task.h
 *
 * Author: Viet Khoa
 */

#ifndef INC_RC_TASK_H_
#define INC_RC_TASK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Channel and timeout configuration */
#define RC_CHANNEL_COUNT             16U
#define RC_WAIT_TIMEOUT_MS           5U
#define RC_LINK_TIMEOUT_MS           100U
#define RC_STREAM_GAP_TIMEOUT_MS     10U
#define RC_RX_MAX_AGE_MS             20U
#define RC_RECOVERY_DELAY_MS         100U

/* RC link state */
typedef enum
{
    RC_LINK_WAITING = 0,
    RC_LINK_ACTIVE,
    RC_LINK_LOST,
    RC_LINK_RX_FAULT
} RC_LinkState_t;

/* Published RC channel snapshot */
typedef struct
{
    uint16_t channels[RC_CHANNEL_COUNT];  /* Decoded RC channel values. */
    uint32_t last_valid_frame_tick;       /* Oldest UART event tick in the batch. */
    uint32_t sequence;                    /* Published frame counter. */
    uint32_t age_ms;                      /* Frame age in milliseconds. */
    RC_LinkState_t link_state;            /* Current RC link state. */
    uint8_t has_frame;                    /* At least one published frame. */
} RC_Snapshot_t;

/* Process RC events, link status and receiver recovery. */
void RCTask_Run(void *argument);

/* Record the oldest queued RX event and notify the task. */
void RCTask_NotifyRxAvailableFromISR(void);

/* Latch a receive error and notify the task. */
void RCTask_NotifyRxErrorFromISR(void);

/* Copy RC status and return its current freshness. */
uint8_t RCTask_GetSnapshot(RC_Snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_RC_TASK_H_ */
