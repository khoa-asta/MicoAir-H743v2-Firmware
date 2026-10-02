#include "rtos_timing.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct {
    uint32_t count;
    uint32_t first;
    uint32_t last;
    uint32_t min_gap;
    uint32_t max_gap;
} TimingAccumulator_t;

static TimingAccumulator_t points[TIMING_POINT_COUNT];
static uint32_t active;
static uint32_t start_tick;
static uint32_t age_count;
static uint32_t age_max;

volatile TimingReport_t dbg_timing_baseline;
volatile TimingReport_t dbg_timing_load;

/**
  * @brief  Starts a new diagnostic window.
  * @param  None
  * @retval None
  */
void RTOSTiming_Begin(void)
{
    taskENTER_CRITICAL();
    memset(points, 0, sizeof(points));
    age_count = 0U;
    age_max = 0U;
    start_tick = osKernelGetTickCount();
    active = 1U;
    taskEXIT_CRITICAL();
}

/**
  * @brief  Records a task event using the kernel tick clock.
  * @param  point Event identifier.
  * @retval None
  */
void RTOSTiming_Record(TimingPoint_t point)
{
    uint32_t now;
    uint32_t gap;
    TimingAccumulator_t *p;

    if ((uint32_t)point >= TIMING_POINT_COUNT) { return; }
    taskENTER_CRITICAL();
    if (active != 0U)
    {
        now = osKernelGetTickCount();
        p = &points[point];
        if (p->count == 0U)
        {
            p->first = now;
        }
        else
        {
            gap = now - p->last;
            if ((p->count == 1U) || (gap < p->min_gap)) { p->min_gap = gap; }
            if (gap > p->max_gap) { p->max_gap = gap; }
        }
        p->last = now;
        p->count++;
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  Records FIFO batch age, measured at the RC chunk read.
  * @param  age_ticks Age from the existing RC timestamp.
  * @retval None
  */
void RTOSTiming_RecordRxAge(uint32_t age_ticks)
{
    taskENTER_CRITICAL();
    if (active != 0U)
    {
        age_count++;
        if (age_ticks > age_max) { age_max = age_ticks; }
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  Freezes the window and publishes its report.
  * @param  out Destination retained for debugger inspection.
  * @retval None
  */
void RTOSTiming_End(volatile TimingReport_t *out)
{
    uint32_t end_tick;
    uint32_t elapsed;
    uint32_t freq;

    if (out == NULL) { return; }
    taskENTER_CRITICAL();
    active = 0U;
    end_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();

    /* No writer can update the accumulators until the next Begin. */
    elapsed = end_tick - start_tick;
    freq = osKernelGetTickFreq();
    out->tick_hz = freq;
    out->elapsed_ticks = elapsed;
    out->rx_age_count = age_count;
    out->rx_age_max_ticks = age_max;
    for (uint32_t i = 0U; i < TIMING_POINT_COUNT; i++)
    {
        const TimingAccumulator_t *p = &points[i];
        out->point[i].count = p->count;
        out->point[i].rate_hz = (elapsed != 0U) ?
            (float)p->count * (float)freq / (float)elapsed : 0.0f;
        out->point[i].gap_count = (p->count > 0U) ? p->count - 1U : 0U;
        out->point[i].gap_min_ticks = p->min_gap;
        out->point[i].gap_max_ticks = p->max_gap;
        out->point[i].first_offset_ticks = (p->count != 0U) ?
            p->first - start_tick : elapsed;
        out->point[i].tail_gap_ticks = (p->count != 0U) ?
            end_tick - p->last : elapsed;
    }
}
