/*
 * bmi088.h
 *
 * BMI088 driver interface for STM32H743.
 *
 * Blocking SPI access is kept private inside bmi088.c and is used only for
 * device initialization and register configuration. Runtime accelerometer and
 * gyroscope acquisition is performed through SPI2 DMA.
 *
 * Author: Viết Khoa
 */

#ifndef DEVICE_BMI088_BMI088_H_
#define DEVICE_BMI088_BMI088_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_RawData_t;

typedef struct
{
    float x;
    float y;
    float z;
} BMI088_Data_t;

typedef enum
{
    BMI088_SPI_IDLE = 0U,
    BMI088_SPI_ACC_DMA,
    BMI088_SPI_GYRO_DMA
} BMI088_SPI_State_t;

/* Exported functions prototypes --------------------------------------------*/

/**
  * @brief  Initializes the BMI088 accelerometer, gyroscope and data-ready outputs.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Init(void);

/**
  * @brief  Starts an SPI2 DMA transaction for one accelerometer XYZ sample.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Accel_ReadRaw_DMA_Start(void);

/**
  * @brief  Gets the completed accelerometer DMA sample and clears its done flag.
  * @param  raw Pointer to the raw accelerometer output structure.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Accel_ReadRaw_DMA_Get(BMI088_RawData_t *raw);

/**
  * @brief  Starts an SPI2 DMA transaction for one gyroscope XYZ sample.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Gyro_ReadRaw_DMA_Start(void);

/**
  * @brief  Gets the completed gyroscope DMA sample and clears its done flag.
  * @param  raw Pointer to the raw gyroscope output structure.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Gyro_ReadRaw_DMA_Get(BMI088_RawData_t *raw);

/**
  * @brief  Returns whether the accelerometer DMA sample is ready to be consumed.
  * @retval 1 if a completed sample is available, otherwise 0.
  */
uint8_t BMI088_Accel_DMA_IsDone(void);

/**
  * @brief  Returns whether the gyroscope DMA sample is ready to be consumed.
  * @retval 1 if a completed sample is available, otherwise 0.
  */
uint8_t BMI088_Gyro_DMA_IsDone(void);

/**
  * @brief  Returns and clears the accelerometer DMA error flag.
  * @retval Previous accelerometer DMA error flag value.
  */
uint8_t BMI088_Accel_DMA_GetAndClearError(void);

/**
  * @brief  Returns and clears the gyroscope DMA error flag.
  * @retval Previous gyroscope DMA error flag value.
  */
uint8_t BMI088_Gyro_DMA_GetAndClearError(void);

/**
  * @brief  Converts raw accelerometer data to g using the configured ±24 g range.
  * @param  raw Pointer to raw accelerometer data.
  * @param  accel_g Pointer to converted accelerometer data in g.
  */
void BMI088_Accel_ConvertToG(const BMI088_RawData_t *raw, BMI088_Data_t *accel_g);

/**
  * @brief  Converts raw gyroscope data to degree/s using the configured ±2000 dps range.
  * @param  raw Pointer to raw gyroscope data.
  * @param  gyro_dps Pointer to converted gyroscope data in degree/s.
  */
void BMI088_Gyro_ConvertToDps(const BMI088_RawData_t *raw, BMI088_Data_t *gyro_dps);

/**
  * @brief  Returns whether SPI2 is currently owned by an active BMI088 DMA transfer.
  * @retval 1 if SPI2 is busy, otherwise 0.
  */
uint8_t BMI088_SPI_IsBusy(void);

/**
  * @brief  Returns the current BMI088 SPI2 state.
  * @retval Current BMI088 SPI state.
  */
BMI088_SPI_State_t BMI088_SPI_GetState(void);

/**
  * @brief  Handles the HAL SPI transmit/receive DMA complete callback for BMI088.
  * @param  hspi Pointer to the SPI handle provided by HAL.
  */
void BMI088_SPI_TxRxCpltHandler(SPI_HandleTypeDef *hspi);

/**
  * @brief  Handles the HAL SPI error callback for BMI088.
  * @param  hspi Pointer to the SPI handle provided by HAL.
  */
void BMI088_SPI_ErrorHandler(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_BMI088_BMI088_H_ */
