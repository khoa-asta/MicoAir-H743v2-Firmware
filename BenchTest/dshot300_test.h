#ifndef DSHOT300_TEST_H
#define DSHOT300_TEST_H

#include "dshot300.h"

typedef struct {
    uint32_t done;
    uint32_t software_pass;
    uint32_t failure_step;
    uint32_t frames_sent;
    HAL_StatusTypeDef result;
} DSHOT300_TestDebug_t;

extern volatile DSHOT300_TestDebug_t dbg_dshot300_test;

/* Run before the scheduler; HAL tick and DMA IRQs must be enabled. */
HAL_StatusTypeDef DSHOT300_TXBench_Run(void);

#endif
