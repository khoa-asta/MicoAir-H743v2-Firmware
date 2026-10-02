#ifndef RATE_ESTIMATOR_CONFIG_H
#define RATE_ESTIMATOR_CONFIG_H

#define RATE_EST_DT_S                 0.001f
#define RATE_EST_IMU_TIMEOUT_MS       5U
#define RATE_EST_AID_TIMEOUT_MS       200U
#define RATE_EST_SETTLE_MS            1000U
#define RATE_EST_COLLECT_MS           2000U
#define RATE_EST_MIN_GYRO_SAMPLES     1800U
#define RATE_EST_MIN_ACCEL_SAMPLES    1400U
#define RATE_EST_MIN_BARO_SAMPLES     60U
#define RATE_EST_MIN_MAG_SAMPLES      60U
#define RATE_EST_START_GYRO_MAX       0.12f
#define RATE_EST_START_ACCEL_GATE     0.8f
#define RATE_EST_START_ACCEL_CHANGE   0.5f
#define RATE_EST_GYRO_STD_MAX         0.015f
#define RATE_EST_ACCEL_STD_MAX        0.20f

/* Physical assumption for THIS bench checkpoint: motors off, board at rest
 * or gently tilted and then held still. Norm/rate gates do not establish
 * low translational acceleration in flight. Replace this policy before flight. */
#define RATE_EST_BENCH_GRAVITY_AID     1U

/* No validated QMC mapping/calibration was supplied. Keep 0 for checkpoint 2B.
 * To enable later: supply BOTH offset and calibrated sensor->body FRD matrix,
 * verify orientation/units, set 1, and reset to collect a new startup window.
 * Identity below is a placeholder, not the confirmed board mapping. */
#define RATE_EST_MAG_CALIBRATED       0U
#define RATE_EST_MAG_OFFSET_UT        {0.0f, 0.0f, 0.0f}
#define RATE_EST_MAG_TO_BODY          {1.0f, 0.0f, 0.0f, \
                                      0.0f, 1.0f, 0.0f, \
                                      0.0f, 0.0f, 1.0f}
#endif
