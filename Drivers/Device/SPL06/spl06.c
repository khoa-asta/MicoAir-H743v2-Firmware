/*
 * spl06.c
 *
 *
 * Author: Viết Khoa
 */

/* Includes */
#include "spl06.h"
#include "i2c.h"
#include <math.h>
#include <stddef.h>

/* Private defines */
#define SPL06_I2C_ADDR_76                       ((uint16_t)(0x76U << 1U))
#define SPL06_I2C_ADDR_77                       ((uint16_t)(0x77U << 1U))
#define SPL06_I2C_TIMEOUT_MS                    50U

#define SPL06_PSR_B2_REG                        0x00U
#define SPL06_PRS_CFG_REG                       0x06U
#define SPL06_TMP_CFG_REG                       0x07U
#define SPL06_MEAS_CFG_REG                      0x08U
#define SPL06_CFG_REG                           0x09U
#define SPL06_RESET_REG                         0x0CU
#define SPL06_ID_REG                            0x0DU
#define SPL06_COEF_START_REG                    0x10U
#define SPL06_COEF_SRCE_REG                     0x28U
#define SPL06_COEF_LENGTH                       18U

#define SPL06_PRODUCT_ID_MASK                   0xF0U
#define SPL06_PRODUCT_ID_VALUE                  0x10U
#define SPL06_RESET_SOFT_CMD                    0x09U

#define SPL06_MEAS_CFG_COEF_RDY_BIT             0x80U
#define SPL06_MEAS_CFG_SENSOR_RDY_BIT           0x40U
#define SPL06_MEAS_CFG_TMP_RDY_BIT              0x20U
#define SPL06_MEAS_CFG_PRS_RDY_BIT              0x10U
#define SPL06_TMP_EXT_BIT                       0x80U

/* Pressure: 64 Hz, 4x oversampling. Temperature: 8 Hz, 1x oversampling. Continuous pressure and temperature mode. */
#define SPL06_PRS_CFG_VALUE                     0x62U
#define SPL06_TMP_CFG_BASE_VALUE                0x30U
#define SPL06_CFG_REG_VALUE                     0x00U
#define SPL06_MEAS_CFG_IDLE                     0x00U
#define SPL06_MEAS_CFG_CONT_PRESS_TEMP          0x07U
#define SPL06_PRESSURE_SCALE_FACTOR             3670016.0f
#define SPL06_TEMPERATURE_SCALE_FACTOR          524288.0f

#define SPL06_RESET_DELAY_MS                    10U
#define SPL06_INIT_READY_TIMEOUT_MS             100U
#define SPL06_FIRST_DATA_TIMEOUT_MS             250U
#define SPL06_STATUS_POLL_DELAY_MS              2U

/* Private variables */
static SPL06_Calibration_t spl06_calibration;
static uint16_t spl06_i2c_address = 0U;
static uint8_t spl06_initialized = 0U;

/* Private function prototypes */
static HAL_StatusTypeDef SPL06_DetectAddress(void);
static HAL_StatusTypeDef SPL06_ReadReg(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef SPL06_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef SPL06_WriteReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef SPL06_WaitForStatus(uint8_t mask, uint32_t timeout_ms);
static HAL_StatusTypeDef SPL06_ReadCalibration(void);
static HAL_StatusTypeDef SPL06_Configure(void);
static int32_t SPL06_SignExtend(uint32_t value, uint8_t bits);

/* Private functions */

/**
  * @brief  Detects the SPL06 I2C address and verifies the product ID.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_DetectAddress(void)
{
    static const uint16_t address_list[] = {SPL06_I2C_ADDR_76, SPL06_I2C_ADDR_77};
    uint8_t id = 0U;

    for (uint32_t i = 0U; i < (sizeof(address_list) / sizeof(address_list[0])); ++i)
    {
        if ((HAL_I2C_Mem_Read(&hi2c2, address_list[i], SPL06_ID_REG, I2C_MEMADD_SIZE_8BIT, &id, 1U, SPL06_I2C_TIMEOUT_MS) == HAL_OK) && ((id & SPL06_PRODUCT_ID_MASK) == SPL06_PRODUCT_ID_VALUE))
        {
            spl06_i2c_address = address_list[i];
            return HAL_OK;
        }
    }

    spl06_i2c_address = 0U;
    return HAL_ERROR;
}

/**
  * @brief  Reads one SPL06 register using blocking I2C2.
  * @param  reg Register address.
  * @param  value Pointer to the received register value.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_ReadReg(uint8_t reg, uint8_t *value)
{
    if ((value == NULL) || (spl06_i2c_address == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2, spl06_i2c_address, reg, I2C_MEMADD_SIZE_8BIT, value, 1U, SPL06_I2C_TIMEOUT_MS);
}

/**
  * @brief  Reads consecutive SPL06 registers using blocking I2C2.
  * @param  start_reg First register address.
  * @param  data Pointer to the receive buffer.
  * @param  length Number of bytes to read.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) || (spl06_i2c_address == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2, spl06_i2c_address, start_reg, I2C_MEMADD_SIZE_8BIT, data, length, SPL06_I2C_TIMEOUT_MS);
}

/**
  * @brief  Writes one SPL06 register using blocking I2C2.
  * @param  reg Register address.
  * @param  value Register value.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_WriteReg(uint8_t reg, uint8_t value)
{
    if (spl06_i2c_address == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(&hi2c2, spl06_i2c_address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, SPL06_I2C_TIMEOUT_MS);
}

/**
  * @brief  Waits until the requested SPL06 status bits are set.
  * @param  mask Required status bits.
  * @param  timeout_ms Maximum wait time in milliseconds.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_WaitForStatus(uint8_t mask, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t status = 0U;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if ((SPL06_ReadReg(SPL06_MEAS_CFG_REG, &status) == HAL_OK) && ((status & mask) == mask))
        {
            return HAL_OK;
        }

        HAL_Delay(SPL06_STATUS_POLL_DELAY_MS);
    }

    return HAL_TIMEOUT;
}

/**
  * @brief  Sign-extends a sensor value to signed 32-bit format.
  * @param  value Unsigned source value.
  * @param  bits Source bit width.
  * @retval Sign-extended value.
  */
static int32_t SPL06_SignExtend(uint32_t value, uint8_t bits)
{
    const uint32_t sign_bit = 1UL << (bits - 1U);
    const uint32_t value_mask = (1UL << bits) - 1UL;

    value &= value_mask;
    if ((value & sign_bit) != 0U)
    {
        value |= ~value_mask;
    }

    return (int32_t)value;
}

/**
  * @brief  Reads and decodes the SPL06 factory calibration coefficients.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_ReadCalibration(void)
{
    uint8_t coef[SPL06_COEF_LENGTH] = {0U};

    if (SPL06_ReadRegs(SPL06_COEF_START_REG, coef, SPL06_COEF_LENGTH) != HAL_OK)
    {
        return HAL_ERROR;
    }

    spl06_calibration.c0 = (int16_t)SPL06_SignExtend(((uint32_t)coef[0] << 4U) | ((uint32_t)coef[1] >> 4U), 12U);
    spl06_calibration.c1 = (int16_t)SPL06_SignExtend((((uint32_t)coef[1] & 0x0FU) << 8U) | (uint32_t)coef[2], 12U);
    spl06_calibration.c00 = SPL06_SignExtend(((uint32_t)coef[3] << 12U) | ((uint32_t)coef[4] << 4U) | ((uint32_t)coef[5] >> 4U), 20U);
    spl06_calibration.c10 = SPL06_SignExtend((((uint32_t)coef[5] & 0x0FU) << 16U) | ((uint32_t)coef[6] << 8U) | (uint32_t)coef[7], 20U);
    spl06_calibration.c01 = (int16_t)SPL06_SignExtend(((uint32_t)coef[8] << 8U) | (uint32_t)coef[9], 16U);
    spl06_calibration.c11 = (int16_t)SPL06_SignExtend(((uint32_t)coef[10] << 8U) | (uint32_t)coef[11], 16U);
    spl06_calibration.c20 = (int16_t)SPL06_SignExtend(((uint32_t)coef[12] << 8U) | (uint32_t)coef[13], 16U);
    spl06_calibration.c21 = (int16_t)SPL06_SignExtend(((uint32_t)coef[14] << 8U) | (uint32_t)coef[15], 16U);
    spl06_calibration.c30 = (int16_t)SPL06_SignExtend(((uint32_t)coef[16] << 8U) | (uint32_t)coef[17], 16U);

    return HAL_OK;
}

/**
  * @brief  Configures SPL06 pressure, temperature and continuous measurement mode.
  * @retval HAL status.
  */
static HAL_StatusTypeDef SPL06_Configure(void)
{
    HAL_StatusTypeDef status;
    uint8_t coef_source = 0U;
    uint8_t tmp_cfg = SPL06_TMP_CFG_BASE_VALUE;

    /* Stop measurements before changing sensor configuration. */
    status = SPL06_WriteReg(SPL06_MEAS_CFG_REG, SPL06_MEAS_CFG_IDLE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Configure pressure measurement to 64 Hz with 4x oversampling. */
    status = SPL06_WriteReg(SPL06_PRS_CFG_REG, SPL06_PRS_CFG_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Select the temperature source used for the factory calibration coefficients. */
    status = SPL06_ReadReg(SPL06_COEF_SRCE_REG, &coef_source);
    if (status != HAL_OK)
    {
        return status;
    }

    if ((coef_source & SPL06_TMP_EXT_BIT) != 0U)
    {
        tmp_cfg |= SPL06_TMP_EXT_BIT;
    }

    /* Configure temperature measurement to 8 Hz with 1x oversampling. */
    status = SPL06_WriteReg(SPL06_TMP_CFG_REG, tmp_cfg);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Disable result shifting, FIFO and sensor interrupts. */
    status = SPL06_WriteReg(SPL06_CFG_REG, SPL06_CFG_REG_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Start continuous pressure and temperature measurement. */
    return SPL06_WriteReg(SPL06_MEAS_CFG_REG, SPL06_MEAS_CFG_CONT_PRESS_TEMP);
}

/* Exported functions */

/**
  * @brief  Initializes the SPL06 and starts continuous pressure and temperature measurement.
  * @retval HAL status.
  */
HAL_StatusTypeDef SPL06_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t id = 0U;

    spl06_initialized = 0U;
    spl06_i2c_address = 0U;

    /* Detect the active SPL06 I2C address and verify the device family. */
    status = SPL06_DetectAddress();
    if (status != HAL_OK)
    {
        return status;
    }

    /* Reset the sensor to its default state. */
    status = SPL06_WriteReg(SPL06_RESET_REG, SPL06_RESET_SOFT_CMD);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(SPL06_RESET_DELAY_MS);

    /* Wait until the sensor and factory coefficients are ready. */
    status = SPL06_WaitForStatus(SPL06_MEAS_CFG_SENSOR_RDY_BIT | SPL06_MEAS_CFG_COEF_RDY_BIT, SPL06_INIT_READY_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Verify the product ID after reset. */
    status = SPL06_ReadReg(SPL06_ID_REG, &id);
    if (status != HAL_OK)
    {
        return status;
    }

    if ((id & SPL06_PRODUCT_ID_MASK) != SPL06_PRODUCT_ID_VALUE)
    {
        return HAL_ERROR;
    }

    /* Load the factory calibration coefficients. */
    status = SPL06_ReadCalibration();
    if (status != HAL_OK)
    {
        return status;
    }

    /* Apply the flight-stack measurement configuration. */
    status = SPL06_Configure();
    if (status != HAL_OK)
    {
        return status;
    }

    /* Wait for the first valid pressure and temperature results. */
    status = SPL06_WaitForStatus(SPL06_MEAS_CFG_PRS_RDY_BIT | SPL06_MEAS_CFG_TMP_RDY_BIT, SPL06_FIRST_DATA_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    spl06_initialized = 1U;
    return HAL_OK;
}

/**
  * @brief  Reads and compensates the latest SPL06 pressure and temperature sample.
  * @param  data Pointer to compensated SPL06 data.
  * @retval HAL status.
  */
HAL_StatusTypeDef SPL06_Read(SPL06_Data_t *data)
{
    uint8_t raw_data[6] = {0U};
    SPL06_RawData_t raw;
    float pressure_scaled;
    float temperature_scaled;
    HAL_StatusTypeDef status;

    if ((data == NULL) || (spl06_initialized == 0U))
    {
        return HAL_ERROR;
    }

    /* Read pressure and temperature result registers in one I2C burst. */
    status = SPL06_ReadRegs(SPL06_PSR_B2_REG, raw_data, sizeof(raw_data));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Decode signed 24-bit pressure and temperature results. */
    raw.pressure_raw = SPL06_SignExtend(((uint32_t)raw_data[0] << 16U) | ((uint32_t)raw_data[1] << 8U) | (uint32_t)raw_data[2], 24U);
    raw.temperature_raw = SPL06_SignExtend(((uint32_t)raw_data[3] << 16U) | ((uint32_t)raw_data[4] << 8U) | (uint32_t)raw_data[5], 24U);

    /* Scale raw pressure and temperature for factory compensation. */
    pressure_scaled = (float)raw.pressure_raw / SPL06_PRESSURE_SCALE_FACTOR;
    temperature_scaled = (float)raw.temperature_raw / SPL06_TEMPERATURE_SCALE_FACTOR;

    /* Apply the SPL06 factory temperature compensation equation. */
    data->temperature_c = ((float)spl06_calibration.c0 * 0.5f) + ((float)spl06_calibration.c1 * temperature_scaled);

    /* Apply the SPL06 factory pressure compensation equation. */
    data->pressure_pa = (float)spl06_calibration.c00 + pressure_scaled * ((float)spl06_calibration.c10 + pressure_scaled * ((float)spl06_calibration.c20 + pressure_scaled * (float)spl06_calibration.c30)) + temperature_scaled * (float)spl06_calibration.c01 + temperature_scaled * pressure_scaled * ((float)spl06_calibration.c11 + pressure_scaled * (float)spl06_calibration.c21);

    return HAL_OK;
}

/**
  * @brief  Converts pressure to altitude relative to a reference pressure.
  * @param  pressure_pa Current pressure in Pa.
  * @param  reference_pressure_pa Reference pressure in Pa.
  * @retval Relative altitude in meters, or NAN for invalid pressure input.
  */
float SPL06_PressureToAltitude(float pressure_pa, float reference_pressure_pa)
{
    if ((pressure_pa <= 0.0f) || (reference_pressure_pa <= 0.0f))
    {
        return NAN;
    }

    return 44330.0f * (1.0f - powf(pressure_pa / reference_pressure_pa, 0.19029495f));
}
