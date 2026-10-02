/*
 * qmc5883l.c
 *
 *      Author: Viết Khoa
 */

/* Include */
#include "qmc5883l.h"
#include "i2c.h"

/* Private defines */
#define QMC5883L_I2C_ADDR                       (0x0DU << 1)

#define QMC5883L_X_LSB_REG                      0x00U
#define QMC5883L_STATUS_REG	                    0x06U
#define QMC5883L_CONTROL_1_REG                  0x09U
#define QMC5883L_CONTROL_2_REG                  0x0AU
#define QMC5883L_SET_RESET_PERIOD_REG           0x0BU
#define QMC5883L_CHIP_ID_REG                    0x0DU

#define QMC5883L_CHIP_ID_VALUE                  0xFFU

#define QMC5883L_STATUS_DRDY                    (1U << 0)
#define QMC5883L_STATUS_OVL                     (1U << 1)

#define QMC5883L_CONTROL_1_VALUE                0x05U
#define QMC5883L_CONTROL_2_VALUE                0x01U
#define QMC5883L_SET_RESET_VALUE                0x01U

#define QMC5883L_SENSITIVITY_LSB_G              12000.0f
#define QMC5883L_GAUSS_TO_UT                    100.0f

#define QMC5883L_I2C_TIMEOUT_MS                 10U

/* Private function prototypes */
static HAL_StatusTypeDef QMC5883L_ReadReg(uint8_t reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef QMC5883L_WriteReg(uint8_t reg, uint8_t value);

/* Private functions */

/**
 * @brief Read QMC5883L register using blocking I2C2
 * @param reg Register address
 * @param data Pointer to receive buffer
 * @param length Number of bytes to read
 */
static HAL_StatusTypeDef QMC5883L_ReadReg(uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, QMC5883L_I2C_TIMEOUT_MS);
}

/**
 * @brief Write QMC5883L register using blocking I2C2
 * @param reg Register address
 * @param value Value to write
 */
static HAL_StatusTypeDef QMC5883L_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, QMC5883L_I2C_TIMEOUT_MS);
}

/**
 * @brief Initialize QMC5883L
 */
HAL_StatusTypeDef QMC5883L_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;

    status = QMC5883L_ReadReg(QMC5883L_CHIP_ID_REG, &chip_id, 1U);
    if (status != HAL_OK)
    {
        return status;
    }

    if (chip_id != QMC5883L_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    status = QMC5883L_WriteReg(QMC5883L_CONTROL_2_REG, QMC5883L_CONTROL_2_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    status = QMC5883L_WriteReg(QMC5883L_SET_RESET_PERIOD_REG, QMC5883L_SET_RESET_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    status = QMC5883L_WriteReg(QMC5883L_CONTROL_1_REG, QMC5883L_CONTROL_1_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(20U);

    return HAL_OK;
}

/**
 * @brief Read magnetic field data from QMC5883L
 * @param data Pointer to output data
 */
HAL_StatusTypeDef QMC5883L_Read(QMC5883L_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t sensor_status;
    uint8_t buffer[6];

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    status = QMC5883L_ReadReg(QMC5883L_STATUS_REG, &sensor_status, 1U);
    if (status != HAL_OK)
    {
        return status;
    }

    if ((sensor_status & QMC5883L_STATUS_OVL) != 0U)
    {
        return HAL_ERROR;
    }

    if ((sensor_status & QMC5883L_STATUS_DRDY) == 0U)
    {
        return HAL_BUSY;
    }

    status = QMC5883L_ReadReg(QMC5883L_X_LSB_REG, buffer, sizeof(buffer));
    if (status != HAL_OK)
    {
        return status;
    }

    data->x_raw = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
    data->y_raw = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
    data->z_raw = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);

    data->x_uT = ((float)data->x_raw / QMC5883L_SENSITIVITY_LSB_G) * QMC5883L_GAUSS_TO_UT;
    data->y_uT = ((float)data->y_raw / QMC5883L_SENSITIVITY_LSB_G) * QMC5883L_GAUSS_TO_UT;
    data->z_uT = ((float)data->z_raw / QMC5883L_SENSITIVITY_LSB_G) * QMC5883L_GAUSS_TO_UT;

    return HAL_OK;
}

