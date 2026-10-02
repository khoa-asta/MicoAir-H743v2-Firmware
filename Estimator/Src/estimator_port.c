#include "estimator_port.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool MapMag(const EstimatorPort_Config_t *config, const float sensor_ut[3], float body_ut[3])
{
    if (config == NULL || sensor_ut == NULL || !config->mag_calibrated)
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        body_ut[i] = 0.0f;
        for (int j = 0; j < 3; ++j)
        {
            body_ut[i] += config->mag_to_body[3 * i + j] * (sensor_ut[j] - config->mag_offset_ut[j]);
        }
        if (!isfinite(body_ut[i]))
        {
            return false;
        }
    }
    return true;
}

bool EstimatorPort_Init(EstimatorPort_t *port, const Estimator_Config_t *filter_config, const EstimatorPort_Config_t *port_config, const float accel_mean_mps2[3], const float gyro_mean_rad_s[3], const float mag_sensor_mean_ut[3], float reference_pressure_pa)
{
    float mag_body[3];
    const float *mag = NULL;

    if (port == NULL || port_config == NULL || !isfinite(reference_pressure_pa) || reference_pressure_pa < 30000.0f || reference_pressure_pa > 120000.0f)
    {
        return false;
    }
    if (port_config->mag_calibrated)
    {
        if (!MapMag(port_config, mag_sensor_mean_ut, mag_body))
        {
            return false;
        }
        mag = mag_body;
    }
    if (!Estimator_Init(&port->filter, filter_config, accel_mean_mps2, gyro_mean_rad_s, mag, 0.0f))
    {
        return false;
    }
    port->config = *port_config;
    port->reference_pressure_pa = reference_pressure_pa;
    port->gyro_sequence = 0U;
    port->accel_sequence = 0U;
    port->held_accel_age_s = 0.0f;
    port->gravity_interval_s = 0.0f;
    port->have_sequence = false;
    port->needs_resync = false;
    return true;
}

bool EstimatorPort_ProcessImu(EstimatorPort_t *port, float dt_s, bool low_dynamics)
{
    IMUTask_Sample_t gyro;
    IMUTask_Sample_t accel;
    if (!IMUTask_GetGyroSample(&gyro) || !IMUTask_GetAccSample(&accel))
    {
        return false;
    }
    return EstimatorPort_ProcessImuSamples(port, &gyro, &accel, dt_s, low_dynamics);
}

bool EstimatorPort_ProcessImuSamples(EstimatorPort_t *port, const IMUTask_Sample_t *gyro, const IMUTask_Sample_t *accel, float dt_s, bool low_dynamics)
{
    float gyro_xyz[3];
    float accel_xyz[3];
    bool new_accel;

    if (port == NULL || !port->filter.initialized || port->needs_resync || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > port->filter.config.max_dt_s)
    {
        return false;
    }
    if (gyro == NULL || accel == NULL || gyro->valid == 0U || accel->valid == 0U)
    {
        return false;
    }
    if (!port->have_sequence)
    {
        /* Chi dat moc dau; chua co khoang cach tu mau truoc de tich phan. */
        port->gyro_sequence = gyro->sequence;
        port->accel_sequence = accel->sequence;
        port->have_sequence = true;
        return false;
    }
    if (gyro->sequence == port->gyro_sequence)
    {
        return false;
    }
    if ((uint32_t)(gyro->sequence - port->gyro_sequence) != 1U)
    {
        /* Getter moi nhat khong khoi phuc duoc cac mau gyro da mat. */
        port->needs_resync = true;
        return false;
    }
    new_accel = accel->sequence != port->accel_sequence;
    port->held_accel_age_s = new_accel ? 0.0f : port->held_accel_age_s + dt_s;
    if (port->held_accel_age_s > 0.005f)
    {
        port->needs_resync = true;
        return false;
    }
    gyro_xyz[0] = gyro->x;
    gyro_xyz[1] = gyro->y;
    gyro_xyz[2] = gyro->z;
    accel_xyz[0] = accel->x;
    accel_xyz[1] = accel->y;
    accel_xyz[2] = accel->z;
    if (!Estimator_Predict(&port->filter, gyro_xyz, accel_xyz, dt_s))
    {
        port->needs_resync = true;
        return false;
    }
    port->gravity_interval_s += dt_s;
    if (new_accel && port->gravity_interval_s >= 0.02f)
    {
        (void)Estimator_UpdateAccel(&port->filter, accel_xyz, low_dynamics);
        port->gravity_interval_s = 0.0f;
    }
    port->gyro_sequence = gyro->sequence;
    port->accel_sequence = accel->sequence;
    return port->filter.healthy;
}

bool EstimatorPort_ProcessMag(EstimatorPort_t *port, const QMC5883L_Data_t *sample)
{
    float sensor[3];
    float body[3];

    if (port == NULL || sample == NULL || port->needs_resync)
    {
        return false;
    }
    sensor[0] = sample->x_uT;
    sensor[1] = sample->y_uT;
    sensor[2] = sample->z_uT;
    return MapMag(&port->config, sensor, body) && Estimator_UpdateMag(&port->filter, body);
}

bool EstimatorPort_ProcessBaro(EstimatorPort_t *port, const SPL06_Data_t *sample)
{
    if (port == NULL || sample == NULL || port->needs_resync || !isfinite(sample->pressure_pa) || sample->pressure_pa < 30000.0f || sample->pressure_pa > 120000.0f)
    {
        return false;
    }
    const float altitude = SPL06_PressureToAltitude(sample->pressure_pa, port->reference_pressure_pa);
    return Estimator_UpdateBaro(&port->filter, altitude);
}
