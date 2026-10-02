#ifndef RC_LOAD_TEST_H
#define RC_LOAD_TEST_H

#include <stdint.h>

typedef struct {
    uint32_t imu_acc;
    uint32_t imu_gyro;
    uint32_t imu_flags;
    uint32_t baro_read;
    uint32_t baro_mutex;
    uint32_t compass_read;
    uint32_t compass_mutex;
    uint32_t rc_stale_bytes;
    uint32_t rc_transport;
    uint32_t rc_flags;
} LoadErrors_t;

extern volatile uint32_t dbg_rc_load_phase;
extern volatile uint32_t dbg_rc_load_done;
extern volatile uint32_t dbg_rc_load_pass;
extern volatile uint32_t dbg_rc_load_failure_code;
extern volatile uint32_t dbg_rc_load_tx_count;
extern volatile uint32_t dbg_rc_load_verified_count;
extern volatile uint32_t dbg_rc_load_deadline_miss_count;
extern volatile uint32_t dbg_rc_load_stack_min_bytes;
extern volatile LoadErrors_t dbg_load_baseline_errors;
extern volatile LoadErrors_t dbg_load_errors;

void RCLoadTest_Run(void *argument);

#endif
