/*
 * rc_task_test.c
 *
 * UART6 loopback only. Remove from the hardware receiver build.
 */

#include "rc_task_test.h"
#include "rc_task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <string.h>

volatile uint32_t dbg_rc_test_done = 0U;
volatile uint32_t dbg_rc_test_pass = 0U;
volatile uint32_t dbg_rc_test_phase = 0U;
volatile uint32_t dbg_rc_test_phases_passed = 0U;
volatile uint32_t dbg_rc_test_failure_code = 0U;
volatile uint32_t dbg_rc_test_tx_count = 0U;
volatile HAL_StatusTypeDef dbg_rc_test_tx_status = HAL_ERROR;

static uint32_t TestTicks(uint32_t ms)
{
    uint32_t ticks = (uint32_t)(((uint64_t)ms * osKernelGetTickFreq() + 999U) / 1000U);
    return (ticks != 0U) ? ticks : 1U;
}

static uint8_t TestCRC(const uint8_t *data, uint16_t size)
{
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < size; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (uint8_t)(((uint32_t)crc << 1U) ^
                           (((crc & 0x80U) != 0U) ? 0xD5U : 0U));
        }
    }
    return crc;
}

static void TestBuildFrame(uint8_t frame[26], uint16_t ch1)
{
    uint16_t channels[RC_CHANNEL_COUNT];
    memset(frame, 0, 26U);
    frame[0] = 0xC8U;
    frame[1] = 24U;
    frame[2] = 0x16U;
    for (uint16_t channel = 0U; channel < RC_CHANNEL_COUNT; channel++)
    {
        channels[channel] = 992U;
    }
    channels[0] = ch1;
    for (uint16_t channel = 0U; channel < RC_CHANNEL_COUNT; channel++)
    {
        for (uint16_t bit = 0U; bit < 11U; bit++)
        {
            uint16_t position = (uint16_t)(channel * 11U + bit);
            if ((channels[channel] & (1U << bit)) != 0U)
            {
                frame[3U + position / 8U] |= (uint8_t)(1U << (position % 8U));
            }
        }
    }
    frame[25] = TestCRC(&frame[2], 23U);
}

static uint8_t TestSend(uint8_t *data, uint16_t size)
{
    dbg_rc_test_tx_status = HAL_UART_Transmit(&huart6, data, size, 20U);
    if (dbg_rc_test_tx_status != HAL_OK)
    {
        dbg_rc_test_failure_code = dbg_rc_test_phase * 10U + 1U;
        return 0U;
    }
    dbg_rc_test_tx_count++;
    return 1U;
}

static uint8_t TestFresh(uint16_t ch1)
{
    RC_Snapshot_t snapshot;
    if ((RCTask_GetSnapshot(&snapshot) == 0U) || (snapshot.channels[0] != ch1))
    {
        dbg_rc_test_failure_code = dbg_rc_test_phase * 10U + 2U;
        return 0U;
    }
    for (uint32_t i = 1U; i < RC_CHANNEL_COUNT; i++)
    {
        if (snapshot.channels[i] != 992U)
        {
            dbg_rc_test_failure_code = dbg_rc_test_phase * 10U + 2U;
            return 0U;
        }
    }
    return 1U;
}

static uint8_t TestLost(void)
{
    RC_Snapshot_t snapshot;
    if ((RCTask_GetSnapshot(&snapshot) != 0U) || (snapshot.link_state != RC_LINK_LOST))
    {
        dbg_rc_test_failure_code = dbg_rc_test_phase * 10U + 3U;
        return 0U;
    }
    return 1U;
}

void RCTaskTest_Run(void *argument)
{
    uint8_t frame[26];
    uint8_t link_frame[14] = {0xC8U, 12U, 0x14U};
    uint8_t burst[78];
    RC_Snapshot_t before;
    RC_Snapshot_t after;

    (void)argument;
    (void)osDelay(TestTicks(100U));

    dbg_rc_test_phase = 1U;
    TestBuildFrame(frame, 992U);
    for (uint32_t i = 0U; i < 50U; i++)
    {
        if (TestSend(frame, sizeof(frame)) == 0U) { goto done; }
        (void)osDelay(TestTicks(20U));
        if (TestFresh(992U) == 0U) { goto done; }
    }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 2U;
    TestBuildFrame(frame, 1811U);
    if (TestSend(frame, sizeof(frame)) == 0U) { goto done; }
    (void)osDelay(TestTicks(5U));
    if (TestFresh(1811U) == 0U) { goto done; }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 3U;
    (void)RCTask_GetSnapshot(&before);
    frame[25] ^= 1U;
    if (TestSend(frame, sizeof(frame)) == 0U) { goto done; }
    (void)osDelay(TestTicks(5U));
    (void)RCTask_GetSnapshot(&after);
    if ((after.sequence != before.sequence) ||
        (after.last_valid_frame_tick != before.last_valid_frame_tick))
    {
        dbg_rc_test_failure_code = 34U;
        goto done;
    }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 4U;
    link_frame[13] = TestCRC(&link_frame[2], 11U);
    for (uint32_t i = 0U; i < 8U; i++)
    {
        if (TestSend(link_frame, sizeof(link_frame)) == 0U) { goto done; }
        (void)osDelay(TestTicks(20U));
    }
    if (TestLost() == 0U) { goto done; }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 5U;
    TestBuildFrame(frame, 172U);
    if (TestSend(frame, 3U) == 0U) { goto done; }
    (void)osDelay(TestTicks(2U));
    if (TestSend(&frame[3], 7U) == 0U) { goto done; }
    (void)osDelay(TestTicks(2U));
    if (TestSend(&frame[10], 16U) == 0U) { goto done; }
    (void)osDelay(TestTicks(5U));
    if (TestFresh(172U) == 0U) { goto done; }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 6U;
    if (TestSend(frame, 8U) == 0U) { goto done; }
    (void)osDelay(TestTicks(20U));
    TestBuildFrame(frame, 992U);
    if (TestSend(frame, sizeof(frame)) == 0U) { goto done; }
    (void)osDelay(TestTicks(5U));
    if (TestFresh(992U) == 0U) { goto done; }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 7U;
    TestBuildFrame(&burst[0], 172U);
    TestBuildFrame(&burst[26], 992U);
    TestBuildFrame(&burst[52], 1811U);
    (void)RCTask_GetSnapshot(&before);
    if (TestSend(burst, sizeof(burst)) == 0U) { goto done; }
    (void)osDelay(TestTicks(5U));
    if (TestFresh(1811U) == 0U) { goto done; }
    (void)RCTask_GetSnapshot(&after);
    if ((uint32_t)(after.sequence - before.sequence) != 3U)
    {
        dbg_rc_test_failure_code = 74U;
        goto done;
    }
    dbg_rc_test_phases_passed++;

    dbg_rc_test_phase = 8U;
    (void)osDelay(TestTicks(RC_LINK_TIMEOUT_MS + 50U));
    if (TestLost() == 0U) { goto done; }
    dbg_rc_test_phases_passed++;
    dbg_rc_test_pass = 1U;

done:
    dbg_rc_test_done = 1U;
    for (;;)
    {
        (void)osDelay(TestTicks(1000U));
    }
}
