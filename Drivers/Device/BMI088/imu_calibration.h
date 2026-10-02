/*
 * Fixed BMI088 calibration imported from this vehicle's PX4 1.16.2 export.
 * Inputs MUST be the unrotated outputs of the supplied BMI088 conversions.
 * Outputs are body FRD specific force (m/s^2) and angular rate (rad/s).
 * This module does not remove gravity or the residual bias estimated by ESKF.
 */
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Apply only once. Input/output arrays may alias. False leaves output intact. */
bool IMU_Calibration_ApplyAccelG(const float sensor_g[3], float body_mps2[3]);
bool IMU_Calibration_ApplyGyroDps(const float sensor_dps[3], float body_rad_s[3]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */
