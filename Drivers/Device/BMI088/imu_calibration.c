/*
 * Calibration for this MicoAir H743 V2 / BMI088 only.
 * Source export: micoairh743v2(2).params, PX4 1.16.2.
 * Reference commit: 54f0455ffcd755534539a7cf33a09a20bf71d29d.
 * Device IDs: ACC0 6946834, GYRO0 6684690.
 *
 * Current driver XYZ -> PX4 calibration frame: [-Y, -X, -Z].
 * This combines PX4 BMI088's Y/Z sign changes and driver yaw 270 deg.
 * Next: SI conversion, subtract PX4 offset, apply accel scale, board trim.
 * Thermal compensation is disabled in the source export.
 */
#include "imu_calibration.h"
#include "imu_calibration_params.h"
#include <math.h>
#include <stddef.h>

#define IMU_ONE_G_MPS2 9.80665f
#define IMU_DEG_TO_RAD 0.017453292519943295f

static bool IMU_Calibration_InputValid(const float input[3], const float output[3])
{
    return (input != NULL) && (output != NULL) &&
           isfinite(input[0]) && isfinite(input[1]) && isfinite(input[2]);
}

static bool IMU_Calibration_ApplyBoardTrim(const float input[3], float output[3])
{
    float result[3];

    for (unsigned row = 0U; row < 3U; ++row)
    {
        result[row] = imu_mounting_rotation[row][0] * input[0] +
                      imu_mounting_rotation[row][1] * input[1] +
                      imu_mounting_rotation[row][2] * input[2];
        if (!isfinite(result[row]))
        {
            return false;
        }
    }

    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        output[axis] = result[axis];
    }
    return true;
}

bool IMU_Calibration_ApplyAccelG(const float sensor_g[3], float body_mps2[3])
{
    if (!IMU_Calibration_InputValid(sensor_g, body_mps2))
    {
        return false;
    }

    /* Convert the native register frame to PX4's pre-calibration frame. */
    float corrected[3] = {-sensor_g[1] * IMU_ONE_G_MPS2,
                          -sensor_g[0] * IMU_ONE_G_MPS2,
                          -sensor_g[2] * IMU_ONE_G_MPS2};

    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        corrected[axis] = (corrected[axis] - imu_accel_offset_mps2[axis]) *
                          imu_accel_scale_factor[axis];
    }
    return IMU_Calibration_ApplyBoardTrim(corrected, body_mps2);
}

bool IMU_Calibration_ApplyGyroDps(const float sensor_dps[3], float body_rad_s[3])
{
    if (!IMU_Calibration_InputValid(sensor_dps, body_rad_s))
    {
        return false;
    }

    float corrected[3] = {-sensor_dps[1] * IMU_DEG_TO_RAD,
                          -sensor_dps[0] * IMU_DEG_TO_RAD,
                          -sensor_dps[2] * IMU_DEG_TO_RAD};

    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        corrected[axis] -= imu_gyro_offset_rad_s[axis];
    }
    return IMU_Calibration_ApplyBoardTrim(corrected, body_rad_s);
}
