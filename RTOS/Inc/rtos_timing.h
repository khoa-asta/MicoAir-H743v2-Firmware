#ifndef RTOS_TIMING_H
#define RTOS_TIMING_H

#include <stdint.h>

typedef enum {
    TIMING_IMU_ACC = 0,
    TIMING_IMU_GYRO,
    TIMING_BARO_LOOP,
    TIMING_BARO_READ,
    TIMING_COMPASS_LOOP,
    TIMING_COMPASS_READ,
    TIMING_RC_PUBLISH,
    TIMING_POINT_COUNT
} TimingPoint_t;

typedef struct {
    uint32_t count;
    float rate_hz;
    uint32_t gap_count;
    uint32_t gap_min_ticks;
    uint32_t gap_max_ticks;
    uint32_t first_offset_ticks;
    uint32_t tail_gap_ticks;
} TimingResult_t;

typedef struct {
    uint32_t tick_hz;
    uint32_t elapsed_ticks;
    uint32_t rx_age_count;
    uint32_t rx_age_max_ticks;
    TimingResult_t point[TIMING_POINT_COUNT];
} TimingReport_t;

extern volatile TimingReport_t dbg_timing_baseline;
extern volatile TimingReport_t dbg_timing_load;

/* Task context only. Begin/End are owned by the bench task. */
void RTOSTiming_Begin(void);
void RTOSTiming_End(volatile TimingReport_t *out);
void RTOSTiming_Record(TimingPoint_t point);
void RTOSTiming_RecordRxAge(uint32_t age_ticks);

#endif
