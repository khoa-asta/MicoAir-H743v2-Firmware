/*
 * spl06.h
 *
 *
 * Author: Viết Khoa
 */

#ifndef DEVICE_SPL06_SPL06_H_
#define DEVICE_SPL06_SPL06_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "main.h"
#include <stdint.h>

/* Exported types */
typedef struct
{
    int32_t pressure_raw;
    int32_t temperature_raw;
} SPL06_RawData_t;

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} SPL06_Calibration_t;

typedef struct
{
    float pressure_pa;
    float temperature_c;
} SPL06_Data_t;

/* Exported function prototypes */

/**
  * @brief  Initializes the SPL06 and starts continuous pressure and temperature measurement.
  * @retval HAL status.
  */
HAL_StatusTypeDef SPL06_Init(void);

/**
  * @brief  Reads and compensates the latest SPL06 pressure and temperature sample.
  * @param  data Pointer to compensated SPL06 data.
  * @retval HAL status.
  */
HAL_StatusTypeDef SPL06_Read(SPL06_Data_t *data);

/**
  * @brief  Converts pressure to altitude relative to a reference pressure.
  * @param  pressure_pa Current pressure in Pa.
  * @param  reference_pressure_pa Reference pressure in Pa.
  * @retval Relative altitude in meters, or NAN for invalid pressure input.
  */
float SPL06_PressureToAltitude(float pressure_pa, float reference_pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_SPL06_SPL06_H_ */
