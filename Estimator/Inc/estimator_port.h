#ifndef ESTIMATOR_PORT_H
#define ESTIMATOR_PORT_H

#include "estimator.h"
#include "imu_task.h"
#include "qmc5883l.h"
#include "spl06.h"

typedef struct
{
    bool mag_calibrated;           /* Chi true khi da biet mapping va hieu chuan. */
    float mag_offset_ut[3];        /* Offset trong he truc thanh ghi QMC. */
    float mag_to_body[9];          /* Row-major: soft iron + doi truc ve FRD. */
} EstimatorPort_Config_t;

typedef struct
{
    Estimator_t filter;
    EstimatorPort_Config_t config;
    float reference_pressure_pa;
    uint32_t gyro_sequence;
    uint32_t accel_sequence;
    float held_accel_age_s;
    float gravity_interval_s;
    bool have_sequence;
    bool needs_resync;
} EstimatorPort_t;

/* Cac mean la trung binh thu khi dung yen; mag_sensor_mean_ut co the NULL. */
bool EstimatorPort_Init(EstimatorPort_t *port, const Estimator_Config_t *filter_config, const EstimatorPort_Config_t *port_config, const float accel_mean_mps2[3], const float gyro_mean_rad_s[3], const float mag_sensor_mean_ut[3], float reference_pressure_pa);
/* Goi o cung mot task so huu estimator; dt la khoang cach hai mau gyro moi. */
bool EstimatorPort_ProcessImu(EstimatorPort_t *port, float dt_s, bool low_dynamics);
/* Same task, explicit snapshots: no second getter can skip ahead after preemption. */
bool EstimatorPort_ProcessImuSamples(EstimatorPort_t *port, const IMUTask_Sample_t *gyro, const IMUTask_Sample_t *accel, float dt_s, bool low_dynamics);
/* Chi chuyen sample moi khi driver Read tra HAL_OK. Khong goi tu task khac. */
bool EstimatorPort_ProcessMag(EstimatorPort_t *port, const QMC5883L_Data_t *sample);
bool EstimatorPort_ProcessBaro(EstimatorPort_t *port, const SPL06_Data_t *sample);

#endif
