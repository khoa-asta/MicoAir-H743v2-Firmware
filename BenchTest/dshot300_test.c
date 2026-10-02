#include "dshot300_test.h"

/* 0: stop frames. 1: distinct scope patterns; disconnect all ESC signals. */
#define DSHOT300_TEST_DISTINCT_PATTERN 0U
#define DSHOT300_TEST_FRAMES           100U

volatile DSHOT300_TestDebug_t dbg_dshot300_test;

HAL_StatusTypeDef DSHOT300_TXBench_Run(void)
{
    dbg_dshot300_test = (DSHOT300_TestDebug_t){0};
    dbg_dshot300_test.failure_step = 1U;
    HAL_StatusTypeDef result = DSHOT300_Init();
    if (result != HAL_OK) goto finish;

#if DSHOT300_TEST_DISTINCT_PATTERN
    const uint16_t values[4] = {48U, 341U, 1024U, 2047U};
#else
    const uint16_t values[4] = {0U, 0U, 0U, 0U};
#endif
    const uint32_t completed_before = DSHOT300_GetCompletedCount();
    for (uint32_t i = 0; i < DSHOT300_TEST_FRAMES; ++i) {
        dbg_dshot300_test.failure_step = 2U;
        result = DSHOT300_Send(values);
        if (result != HAL_OK) goto finish;
        const uint32_t started = HAL_GetTick();
        dbg_dshot300_test.failure_step = 3U;
        while (DSHOT300_IsBusy()) {
            DSHOT300_Process();
            if (DSHOT300_GetState() == DSHOT300_FAULT) break;
            if ((uint32_t)(HAL_GetTick() - started) >= 20U) {
                (void)DSHOT300_Abort();
                result = HAL_TIMEOUT;
                goto finish;
            }
            HAL_Delay(1U);
        }
        if (DSHOT300_GetState() != DSHOT300_READY) {
            result = HAL_ERROR;
            goto finish;
        }
        ++dbg_dshot300_test.frames_sent;
        HAL_Delay(10U);
    }
    dbg_dshot300_test.failure_step = 4U;
    if ((DSHOT300_GetCompletedCount() - completed_before) != DSHOT300_TEST_FRAMES) {
        result = HAL_ERROR;
        goto finish;
    }
    dbg_dshot300_test.failure_step = 0U;
    dbg_dshot300_test.software_pass = 1U;
    result = HAL_OK;

finish:
    dbg_dshot300_test.result = result;
    dbg_dshot300_test.done = 1U;
    return result;
}
