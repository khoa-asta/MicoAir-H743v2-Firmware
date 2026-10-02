#include "rc_load_test.h"
#include "rtos_timing.h"
#include "rc_task.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>

#define RC_LOAD_RATE_HZ       50U
#define RC_LOAD_SECONDS       30U
#define RC_BASELINE_SECONDS   5U
#define RC_LOAD_FRAME_COUNT   (RC_LOAD_RATE_HZ * RC_LOAD_SECONDS)

_Static_assert(RC_CHANNEL_COUNT == 16U, "CRSF bench requires 16 channels");

extern volatile HAL_StatusTypeDef dbg_rc_init_status;
extern volatile HAL_StatusTypeDef dbg_rc_start_status;
extern volatile uint32_t dbg_imu_acc_error_count;
extern volatile uint32_t dbg_imu_gyro_error_count;
extern volatile uint32_t dbg_imu_thread_flag_error_count;
extern volatile uint32_t dbg_baro_read_error_count;
extern volatile uint32_t dbg_baro_mutex_error_count;
extern volatile uint32_t dbg_compass_read_error_count;
extern volatile uint32_t dbg_compass_mutex_error_count;
extern volatile uint32_t dbg_rc_stale_byte_count;
extern volatile uint32_t dbg_rc_transport_fault_count;
extern volatile uint32_t dbg_rc_thread_flag_error_count;

volatile uint32_t dbg_rc_load_phase;
volatile uint32_t dbg_rc_load_done;
volatile uint32_t dbg_rc_load_pass;
volatile uint32_t dbg_rc_load_failure_code;
volatile uint32_t dbg_rc_load_tx_count;
volatile uint32_t dbg_rc_load_verified_count;
volatile uint32_t dbg_rc_load_deadline_miss_count;
volatile uint32_t dbg_rc_load_stack_min_bytes;
volatile HAL_StatusTypeDef dbg_rc_load_tx_status = HAL_ERROR;
volatile LoadErrors_t dbg_load_baseline_errors;
volatile LoadErrors_t dbg_load_errors;

static LoadErrors_t RCLoad_ReadErrors(void)
{
    LoadErrors_t e;
    e.imu_acc = dbg_imu_acc_error_count;
    e.imu_gyro = dbg_imu_gyro_error_count;
    e.imu_flags = dbg_imu_thread_flag_error_count;
    e.baro_read = dbg_baro_read_error_count;
    e.baro_mutex = dbg_baro_mutex_error_count;
    e.compass_read = dbg_compass_read_error_count;
    e.compass_mutex = dbg_compass_mutex_error_count;
    e.rc_stale_bytes = dbg_rc_stale_byte_count;
    e.rc_transport = dbg_rc_transport_fault_count;
    e.rc_flags = dbg_rc_thread_flag_error_count;
    return e;
}

static void RCLoad_ErrorDelta(volatile LoadErrors_t *out, const LoadErrors_t *before)
{
    LoadErrors_t after = RCLoad_ReadErrors();
#define ERROR_DELTA(member) out->member = after.member - before->member
    ERROR_DELTA(imu_acc);
    ERROR_DELTA(imu_gyro);
    ERROR_DELTA(imu_flags);
    ERROR_DELTA(baro_read);
    ERROR_DELTA(baro_mutex);
    ERROR_DELTA(compass_read);
    ERROR_DELTA(compass_mutex);
    ERROR_DELTA(rc_stale_bytes);
    ERROR_DELTA(rc_transport);
    ERROR_DELTA(rc_flags);
#undef ERROR_DELTA
}

/* Independent CRSF 0x16 encoder: 16 little-endian, 11-bit channels. */
static void RCLoad_BuildFrame(uint8_t frame[26], uint16_t ch1)
{
    uint8_t crc = 0U;
    memset(frame, 0, 26U);
    frame[0] = 0xC8U;
    frame[1] = 24U;
    frame[2] = 0x16U;
    for (uint32_t ch = 0U; ch < 16U; ch++)
    {
        uint16_t value = (ch == 0U) ? ch1 : 992U;
        for (uint32_t bit = 0U; bit < 11U; bit++)
        {
            uint32_t position = ch * 11U + bit;
            if (((value >> bit) & 1U) != 0U)
            {
                frame[3U + position / 8U] |= (uint8_t)(1U << (position % 8U));
            }
        }
    }
    for (uint32_t i = 2U; i < 25U; i++)
    {
        crc ^= frame[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0xD5U) :
                                     (uint8_t)(crc << 1U);
        }
    }
    frame[25] = crc;
}

/**
  * @brief  Measures a quiet baseline and a 30-second RC loopback load.
  * @param  argument Unused.
  * @retval None
  */
void RCLoadTest_Run(void *argument)
{
    uint32_t freq = osKernelGetTickFreq();
    uint32_t period;
    uint32_t next_tick;
    uint32_t before_sequence;
    uint16_t ch1;
    uint8_t frame[26];
    RC_Snapshot_t snapshot;
    LoadErrors_t errors_before;

    (void)argument;
    if ((freq < RC_LOAD_RATE_HZ) || ((freq % RC_LOAD_RATE_HZ) != 0U))
    {
        dbg_rc_load_failure_code = 1U;
        goto finish;
    }
    period = freq / RC_LOAD_RATE_HZ;
    (void)osDelay(freq);
    if ((dbg_rc_init_status != HAL_OK) || (dbg_rc_start_status != HAL_OK))
    {
        dbg_rc_load_failure_code = 2U;
        goto finish;
    }

    dbg_rc_load_phase = 1U;
    errors_before = RCLoad_ReadErrors();
    RTOSTiming_Begin();
    (void)osDelay(freq * RC_BASELINE_SECONDS);
    RTOSTiming_End(&dbg_timing_baseline);
    RCLoad_ErrorDelta(&dbg_load_baseline_errors, &errors_before);
    if (dbg_timing_baseline.point[TIMING_RC_PUBLISH].count != 0U)
    {
        dbg_rc_load_failure_code = 3U;
        goto finish;
    }

    dbg_rc_load_phase = 2U;
    errors_before = RCLoad_ReadErrors();
    RTOSTiming_Begin();
    next_tick = osKernelGetTickCount();
    for (uint32_t i = 0U; i < RC_LOAD_FRAME_COUNT; i++)
    {
        (void)RCTask_GetSnapshot(&snapshot);
        before_sequence = snapshot.sequence;
        ch1 = (uint16_t)(172U + (i % 1640U));
        RCLoad_BuildFrame(frame, ch1);
        dbg_rc_load_tx_status = HAL_UART_Transmit(&huart6, frame, sizeof(frame), 10U);
        if (dbg_rc_load_tx_status != HAL_OK)
        {
            dbg_rc_load_failure_code = 4U;
            break;
        }
        dbg_rc_load_tx_count++;
        next_tick += period;
        if (((int32_t)(next_tick - osKernelGetTickCount()) <= 0) ||
            (osDelayUntil(next_tick) != osOK))
        {
            dbg_rc_load_deadline_miss_count++;
            dbg_rc_load_failure_code = 5U;
            break;
        }
        if ((RCTask_GetSnapshot(&snapshot) == 0U) ||
            ((uint32_t)(snapshot.sequence - before_sequence) != 1U) ||
            (snapshot.channels[0] != ch1))
        {
            dbg_rc_load_failure_code = 6U;
            break;
        }
        for (uint32_t ch = 1U; ch < RC_CHANNEL_COUNT; ch++)
        {
            if (snapshot.channels[ch] != 992U)
            {
                dbg_rc_load_failure_code = 7U;
                break;
            }
        }
        if (dbg_rc_load_failure_code != 0U) { break; }
        dbg_rc_load_verified_count++;
    }
    RTOSTiming_End(&dbg_timing_load);
    RCLoad_ErrorDelta(&dbg_load_errors, &errors_before);
    if ((dbg_rc_load_failure_code == 0U) &&
        ((dbg_load_errors.rc_stale_bytes != 0U) ||
         (dbg_load_errors.rc_transport != 0U) ||
         (dbg_load_errors.rc_flags != 0U)))
    {
        dbg_rc_load_failure_code = 8U;
    }
    if ((dbg_rc_load_failure_code == 0U) &&
        (dbg_timing_load.point[TIMING_RC_PUBLISH].count != RC_LOAD_FRAME_COUNT))
    {
        dbg_rc_load_failure_code = 9U;
    }
    dbg_rc_load_pass = (uint32_t)((dbg_rc_load_failure_code == 0U) &&
        (dbg_rc_load_verified_count == RC_LOAD_FRAME_COUNT));

finish:
    dbg_rc_load_stack_min_bytes = osThreadGetStackSpace(osThreadGetId());
    dbg_rc_load_phase = 3U;
    dbg_rc_load_done = 1U;
    /* Keep the task alive; reports stay frozen after TX stops. */
    for (;;) { (void)osDelay((freq != 0U) ? freq : 1U); }
}
