/*
 * rc_task.c
 *
 * Author: Viet Khoa
 */

/* Includes */
#include "rc_task.h"
#include "elrs.h"
#include "crsf.h"
#include "cmsis_os2.h"
#include "rtos_timing.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/* Task configuration */
#define RC_FLAG_RX_AVAILABLE (1UL << 0)
#define RC_FLAG_RX_ERROR     (1UL << 1)
#define RC_EVENT_MASK        (RC_FLAG_RX_AVAILABLE | RC_FLAG_RX_ERROR)
#define RC_RX_CHUNK_SIZE     64U
#define RC_RX_BUDGET_BYTES   256U

_Static_assert(RC_CHANNEL_COUNT == CRSF_CHANNEL_COUNT, "RC channel count mismatch");

/* RTOS objects */
extern osThreadId_t RCTaskHandle;

/* RC runtime state */
typedef struct
{
    volatile uint8_t task_ready;       /* ISR notification readiness. */
    volatile uint8_t rx_pending;       /* Unread RX batch is tracked. */
    volatile uint8_t rx_error;         /* Latched transport error. */
    volatile uint32_t oldest_rx_tick;  /* Oldest unread RX event tick. */
    uint8_t stream_active;             /* CRSF stream activity. */
    uint8_t recovery_pending;          /* Receiver restart state. */
    uint32_t last_byte_tick;           /* Previous RX event tick. */
    uint32_t tick_freq;                /* RTOS ticks per second. */
    uint32_t link_timeout_ticks;       /* Maximum valid frame age. */
    uint32_t stream_timeout_ticks;     /* Maximum stream event gap. */
    uint32_t max_rx_age_ticks;         /* Maximum queued RX age. */
} RCTask_Runtime_t;

/* Runtime and published RC data */
static RCTask_Runtime_t s_runtime;
static RC_Snapshot_t s_snapshot;

/* Private function prototypes */
static uint32_t RCTask_MsToTicks(uint32_t ms);
static uint32_t RCTask_TicksToMs(uint32_t ticks);
static void RCTask_SetFlagFromISR(uint32_t flag);
static void RCTask_SetState(RC_LinkState_t state);
static void RCTask_ResetStream(void);
static HAL_StatusTypeDef RCTask_StartReceiver(void);
static uint16_t RCTask_ReadChunk(uint8_t *data, uint32_t *rx_tick);
static void RCTask_ProcessRx(void);
static void RCTask_UpdateStatus(void);

/* Convert milliseconds to at least one RTOS tick. */
static uint32_t RCTask_MsToTicks(uint32_t ms)
{
    uint32_t ticks = (uint32_t)(((uint64_t)ms * s_runtime.tick_freq + 999U) / 1000U);

    if (ticks == 0U)
    {
        return 1U;
    }

    return ticks;
}

/* Convert RTOS ticks to milliseconds with saturation. */
static uint32_t RCTask_TicksToMs(uint32_t ticks)
{
    uint64_t milliseconds;

    if (s_runtime.tick_freq == 0U)
    {
        return UINT32_MAX;
    }

    milliseconds = ((uint64_t)ticks * 1000U) / s_runtime.tick_freq;

    if (milliseconds >= UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)milliseconds;
}

/* Set an RC event flag from interrupt context. */
static void RCTask_SetFlagFromISR(uint32_t flag)
{
    if ((s_runtime.task_ready == 0U) || (RCTaskHandle == NULL))
    {
        return;
    }

    (void)osThreadFlagsSet(RCTaskHandle, flag);
}

/* Update the published RC link state. */
static void RCTask_SetState(RC_LinkState_t state)
{
    taskENTER_CRITICAL();
    s_snapshot.link_state = state;
    taskEXIT_CRITICAL();
}

/* Discard the partial CRSF stream. */
static void RCTask_ResetStream(void)
{
    CRSF_ResetStream();
    s_runtime.stream_active = 0U;
}

/* Restart the ELRS transport and reset stream state. */
static HAL_StatusTypeDef RCTask_StartReceiver(void)
{
    HAL_StatusTypeDef status;

    s_runtime.task_ready = 0U;
    status = ELRS_Receive_DMA_Stop();

    if (status != HAL_OK)
    {
        return status;
    }

    status = ELRS_Init();

    if (status != HAL_OK)
    {
        return status;
    }

    (void)osThreadFlagsClear(RC_EVENT_MASK);
    s_runtime.rx_pending = 0U;
    s_runtime.rx_error = 0U;
    RCTask_ResetStream();

    if (s_snapshot.has_frame != 0U)
    {
        RCTask_SetState(RC_LINK_LOST);
    }
    else
    {
        RCTask_SetState(RC_LINK_WAITING);
    }

    s_runtime.task_ready = 1U;
    status = ELRS_Receive_DMA_Start();

    if (status != HAL_OK)
    {
        s_runtime.task_ready = 0U;
    }

    return status;
}

/* Read one FIFO chunk with its oldest RX event tick. */
static uint16_t RCTask_ReadChunk(uint8_t *data, uint32_t *rx_tick)
{
    uint16_t count;

    taskENTER_CRITICAL();

    if (s_runtime.rx_pending == 0U)
    {
        if (ELRS_Available() != 0U)
        {
            s_runtime.rx_error = 1U;
        }

        taskEXIT_CRITICAL();

        return 0U;
    }

    *rx_tick = s_runtime.oldest_rx_tick;
    count = ELRS_Read(data, RC_RX_CHUNK_SIZE);

    if (ELRS_Available() == 0U)
    {
        s_runtime.rx_pending = 0U;
    }

    taskEXIT_CRITICAL();

    return count;
}

/* Decode queued bytes and publish fresh RC channels. */
static void RCTask_ProcessRx(void)
{
    uint8_t data[RC_RX_CHUNK_SIZE];
    uint16_t channels[RC_CHANNEL_COUNT];
    uint32_t processed = 0U;

    while (processed < RC_RX_BUDGET_BYTES)
    {
        uint16_t count;
        uint32_t rx_tick;
        uint32_t now;
        uint32_t age;

        if ((s_runtime.rx_error != 0U) || (ELRS_HasRxFault() != 0U))
        {
            return;
        }

        count = RCTask_ReadChunk(data, &rx_tick);

        if (count == 0U)
        {
            break;
        }

        processed += count;
        now = osKernelGetTickCount();
        age = (uint32_t)(now - rx_tick);
        RTOSTiming_RecordRxAge(age);

        /* Reject expired receive data. */
        if (age >= s_runtime.max_rx_age_ticks)
        {
            RCTask_ResetStream();
            RCTask_SetState(RC_LINK_LOST);
            continue;
        }

        if ((s_runtime.stream_active != 0U) &&
            ((uint32_t)(rx_tick - s_runtime.last_byte_tick) >= s_runtime.stream_timeout_ticks))
        {
            RCTask_ResetStream();
        }

        s_runtime.last_byte_tick = rx_tick;
        s_runtime.stream_active = 1U;

        for (uint16_t i = 0U; i < count; i++)
        {
            uint8_t updates = CRSF_ProcessByte(data[i]);

            if ((updates != 0U) && (CRSF_GetChannels(channels) != 0U))
            {
                taskENTER_CRITICAL();
                now = osKernelGetTickCount();

                if ((s_runtime.rx_error == 0U) &&
                    (ELRS_HasRxFault() == 0U) &&
                    ((uint32_t)(now - rx_tick) < s_runtime.max_rx_age_ticks))
                {
                    /* Publish one complete RC channel snapshot. */
                    memcpy(s_snapshot.channels, channels, sizeof(channels));
                    s_snapshot.last_valid_frame_tick = rx_tick;
                    s_snapshot.sequence += updates;
                    s_snapshot.has_frame = 1U;
                    s_snapshot.link_state = RC_LINK_ACTIVE;
                    RTOSTiming_Record(TIMING_RC_PUBLISH);
                }

                taskEXIT_CRITICAL();
            }
        }
    }
}

/* Copy RC status and return its current freshness. */
uint8_t RCTask_GetSnapshot(RC_Snapshot_t *out)
{
    uint8_t fault;
    uint32_t now;

    if (out == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = s_snapshot;
    fault = (uint8_t)((s_runtime.rx_error != 0U) || (ELRS_HasRxFault() != 0U));
    taskEXIT_CRITICAL();

    out->age_ms = UINT32_MAX;

    if ((s_runtime.tick_freq != 0U) && (out->has_frame != 0U))
    {
        now = osKernelGetTickCount();
        out->age_ms = RCTask_TicksToMs((uint32_t)(now - out->last_valid_frame_tick));

        if ((out->link_state == RC_LINK_ACTIVE) &&
            ((uint32_t)(now - out->last_valid_frame_tick) >= s_runtime.link_timeout_ticks))
        {
            out->link_state = RC_LINK_LOST;
        }
    }

    if (fault != 0U)
    {
        out->link_state = RC_LINK_RX_FAULT;
    }

    if ((out->has_frame != 0U) && (out->link_state == RC_LINK_ACTIVE))
    {
        return 1U;
    }

    return 0U;
}

/* Check RC link and partial-stream timeouts. */
static void RCTask_UpdateStatus(void)
{
    uint32_t now = osKernelGetTickCount();

    if ((s_snapshot.link_state == RC_LINK_ACTIVE) &&
        ((uint32_t)(now - s_snapshot.last_valid_frame_tick) >= s_runtime.link_timeout_ticks))
    {
        RCTask_SetState(RC_LINK_LOST);
    }

    if ((s_runtime.stream_active != 0U) &&
        (ELRS_Available() == 0U) &&
        ((uint32_t)(now - s_runtime.last_byte_tick) >= s_runtime.stream_timeout_ticks))
    {
        RCTask_ResetStream();
    }
}

/* Process RC events, link status and receiver recovery. */
void RCTask_Run(void *argument)
{
    uint32_t wait_ticks;
    uint32_t recovery_ticks;

    (void)argument;

    memset(&s_runtime, 0, sizeof(s_runtime));
    memset(&s_snapshot, 0, sizeof(s_snapshot));

    s_runtime.tick_freq = osKernelGetTickFreq();
    s_runtime.link_timeout_ticks = RCTask_MsToTicks(RC_LINK_TIMEOUT_MS);
    s_runtime.stream_timeout_ticks = RCTask_MsToTicks(RC_STREAM_GAP_TIMEOUT_MS);
    s_runtime.max_rx_age_ticks = RCTask_MsToTicks(RC_RX_MAX_AGE_MS);
    wait_ticks = RCTask_MsToTicks(RC_WAIT_TIMEOUT_MS);
    recovery_ticks = RCTask_MsToTicks(RC_RECOVERY_DELAY_MS);

    CRSF_Init();
    RCTask_SetState(RC_LINK_WAITING);

    if (RCTask_StartReceiver() != HAL_OK)
    {
        s_runtime.recovery_pending = 1U;
        RCTask_SetState(RC_LINK_RX_FAULT);
    }

    for (;;)
    {
        uint32_t flags = 0U;

        /* Retry a failed receiver after the recovery delay. */
        if (s_runtime.recovery_pending != 0U)
        {
            RCTask_UpdateStatus();
            (void)osDelay(recovery_ticks);

            if (RCTask_StartReceiver() == HAL_OK)
            {
                s_runtime.recovery_pending = 0U;
            }
            else
            {
                RCTask_SetState(RC_LINK_RX_FAULT);
            }

            continue;
        }

        if (ELRS_Available() == 0U)
        {
            flags = osThreadFlagsWait(RC_EVENT_MASK, osFlagsWaitAny, wait_ticks);
        }

        if ((flags != osFlagsErrorTimeout) && ((flags & osFlagsError) != 0U))
        {
            RCTask_SetState(RC_LINK_RX_FAULT);
            s_runtime.recovery_pending = 1U;
            continue;
        }

        RCTask_UpdateStatus();
        RCTask_ProcessRx();

        if ((s_runtime.rx_error != 0U) || (ELRS_HasRxFault() != 0U))
        {
            RCTask_SetState(RC_LINK_RX_FAULT);
            s_runtime.recovery_pending = 1U;
            s_runtime.task_ready = 0U;
            (void)ELRS_Receive_DMA_Stop();
            RCTask_ResetStream();
        }

        RCTask_UpdateStatus();

        if ((s_runtime.recovery_pending == 0U) && (ELRS_Available() != 0U))
        {
            (void)osDelay(1U);
        }
    }
}

/* Record the oldest queued RX event and notify the task. */
void RCTask_NotifyRxAvailableFromISR(void)
{
    if ((s_runtime.task_ready == 0U) || (RCTaskHandle == NULL))
    {
        return;
    }

    if (ELRS_Available() != 0U)
    {
        if (s_runtime.rx_pending == 0U)
        {
            s_runtime.oldest_rx_tick = osKernelGetTickCount();
            s_runtime.rx_pending = 1U;
        }

        RCTask_SetFlagFromISR(RC_FLAG_RX_AVAILABLE);
    }

    if (ELRS_HasRxFault() != 0U)
    {
        RCTask_NotifyRxErrorFromISR();
    }
}

/* Latch a receive error and notify the task. */
void RCTask_NotifyRxErrorFromISR(void)
{
    if ((s_runtime.task_ready == 0U) || (RCTaskHandle == NULL))
    {
        return;
    }

    s_runtime.rx_error = 1U;
    RCTask_SetFlagFromISR(RC_FLAG_RX_ERROR);
}
