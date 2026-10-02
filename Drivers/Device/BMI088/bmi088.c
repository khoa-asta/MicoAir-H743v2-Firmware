/*
 * bmi088.c
 *
 * BMI088 driver implementation for STM32H743.
 * Blocking SPI transactions are used for device initialization and register
 * configuration only. Runtime accelerometer and gyroscope data acquisition is
 * performed using SPI2 DMA.
 *
 * Author: Viết Khoa
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "spi.h"

/* Private defines -----------------------------------------------------------*/
#define BMI088_SPI_READ_MASK                     0x80U
#define BMI088_SPI_WRITE_MASK                    0x7FU
#define BMI088_SPI_TIMEOUT_MS                    100U

#define BMI088_DMA_CACHE_LINE_SIZE               32U
#define BMI088_ACC_DMA_TRANSFER_SIZE             8U
#define BMI088_GYRO_DMA_TRANSFER_SIZE            7U

#define BMI088_ACC_CHIP_ID_REG                   0x00U
#define BMI088_ACC_CHIP_ID_VALUE                 0x1EU
#define BMI088_ACC_X_LSB_REG                     0x12U
#define BMI088_ACC_CONF_REG                      0x40U
#define BMI088_ACC_RANGE_REG                     0x41U
#define BMI088_ACC_INT1_IO_CTRL_REG              0x53U
#define BMI088_ACC_INT_MAP_DATA_REG              0x58U
#define BMI088_ACC_PWR_CONF_REG                  0x7CU
#define BMI088_ACC_PWR_CTRL_REG                  0x7DU
#define BMI088_ACC_SOFTRESET_REG                 0x7EU

#define BMI088_GYRO_CHIP_ID_REG                  0x00U
#define BMI088_GYRO_CHIP_ID_VALUE                0x0FU
#define BMI088_GYRO_RATE_X_LSB_REG               0x02U
#define BMI088_GYRO_RANGE_REG                    0x0FU
#define BMI088_GYRO_BANDWIDTH_REG                0x10U
#define BMI088_GYRO_LPM1_REG                     0x11U
#define BMI088_GYRO_SOFTRESET_REG                0x14U
#define BMI088_GYRO_INT_CTRL_REG                 0x15U
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG        0x16U
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG         0x18U

#define BMI088_SOFTRESET_CMD                     0xB6U

/* Accelerometer configuration: active mode, 800 Hz ODR, normal bandwidth and ±24 g range. */
#define BMI088_ACC_ACTIVE                        0x00U
#define BMI088_ACC_ENABLE                        0x04U
#define BMI088_ACC_CONF_800HZ_NORMAL             0xABU
#define BMI088_ACC_RANGE_24G                     0x03U

/* Accelerometer INT1 configuration: output enabled, push-pull, active high, data-ready mapped to INT1. */
#define BMI088_ACC_INT1_ACTIVE_HIGH_PP           0x0AU
#define BMI088_ACC_DRDY_TO_INT1                  0x04U

/* Gyroscope configuration: normal mode, 1000 Hz ODR, 116 Hz bandwidth and ±2000 degree/s range. */
#define BMI088_GYRO_NORMAL_MODE                  0x00U
#define BMI088_GYRO_BW_1000_116                  0x02U
#define BMI088_GYRO_RANGE_2000DPS                0x00U

/* Gyroscope INT3 configuration: new-data interrupt enabled, push-pull, active high and mapped to INT3. */
#define BMI088_GYRO_NEW_DATA_INT_ENABLE          0x80U
#define BMI088_GYRO_INT3_ACTIVE_HIGH_PP          0x0DU
#define BMI088_GYRO_DRDY_TO_INT3                 0x01U

#define BMI088_ACC_SCALE_G                       (24.0f / 32768.0f)
#define BMI088_GYRO_SCALE_DPS                    (2000.0f / 32768.0f)

/* Private variables ---------------------------------------------------------*/
static uint8_t bmi088_acc_dma_tx[BMI088_DMA_CACHE_LINE_SIZE] __attribute__((aligned(BMI088_DMA_CACHE_LINE_SIZE)));
static uint8_t bmi088_acc_dma_rx[BMI088_DMA_CACHE_LINE_SIZE] __attribute__((aligned(BMI088_DMA_CACHE_LINE_SIZE)));
static uint8_t bmi088_gyro_dma_tx[BMI088_DMA_CACHE_LINE_SIZE] __attribute__((aligned(BMI088_DMA_CACHE_LINE_SIZE)));
static uint8_t bmi088_gyro_dma_rx[BMI088_DMA_CACHE_LINE_SIZE] __attribute__((aligned(BMI088_DMA_CACHE_LINE_SIZE)));

static volatile BMI088_SPI_State_t bmi088_spi_state = BMI088_SPI_IDLE;
static volatile uint8_t bmi088_acc_dma_done = 0U;
static volatile uint8_t bmi088_gyro_dma_done = 0U;
static volatile uint8_t bmi088_acc_dma_error = 0U;
static volatile uint8_t bmi088_gyro_dma_error = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BMI088_Accel_Select(void);
static void BMI088_Accel_Deselect(void);
static void BMI088_Gyro_Select(void);
static void BMI088_Gyro_Deselect(void);
static void BMI088_DMA_CleanTxBuffer(uint8_t *buffer);
static void BMI088_DMA_InvalidateRxBuffer(uint8_t *buffer);
static HAL_StatusTypeDef BMI088_Accel_ReadReg(uint8_t reg, uint8_t *data);
static HAL_StatusTypeDef BMI088_Accel_WriteReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data);
static HAL_StatusTypeDef BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef BMI088_DRDY_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Drives the accelerometer chip-select low.
  */
static void BMI088_Accel_Select(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Drives the accelerometer chip-select high.
  */
static void BMI088_Accel_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Drives the gyroscope chip-select low.
  */
static void BMI088_Gyro_Select(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Drives the gyroscope chip-select high.
  */
static void BMI088_Gyro_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Cleans one complete Cortex-M7 cache line containing a DMA TX buffer.
  * @param  buffer Pointer to a 32-byte aligned TX buffer.
  */
static void BMI088_DMA_CleanTxBuffer(uint8_t *buffer)
{
    SCB_CleanDCache_by_Addr((uint32_t *)buffer, (int32_t)BMI088_DMA_CACHE_LINE_SIZE);
}

/**
  * @brief  Invalidates one complete Cortex-M7 cache line containing a DMA RX buffer.
  * @param  buffer Pointer to a 32-byte aligned RX buffer.
  */
static void BMI088_DMA_InvalidateRxBuffer(uint8_t *buffer)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)buffer, (int32_t)BMI088_DMA_CACHE_LINE_SIZE);
}

/**
  * @brief  Reads one accelerometer register using blocking SPI during initialization.
  * @param  reg Accelerometer register address.
  * @param  data Pointer to the received register value.
  * @retval HAL status.
  */
static HAL_StatusTypeDef BMI088_Accel_ReadReg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx[3] = {0U};
    uint8_t rx[3] = {0U};

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    /*
     * Accelerometer SPI reads require one additional protocol dummy byte.
     * The requested register value is therefore received in rx[2].
     */
    tx[0] = reg | BMI088_SPI_READ_MASK;

    BMI088_Accel_Select();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, BMI088_SPI_TIMEOUT_MS);
    BMI088_Accel_Deselect();

    if (status == HAL_OK)
    {
        *data = rx[2];
    }

    return status;
}

/**
  * @brief  Writes one accelerometer register using blocking SPI during initialization.
  * @param  reg Accelerometer register address.
  * @param  data Register value to write.
  * @retval HAL status.
  */
static HAL_StatusTypeDef BMI088_Accel_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx[2];

    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    tx[0] = reg & BMI088_SPI_WRITE_MASK;
    tx[1] = data;

    BMI088_Accel_Select();
    status = HAL_SPI_Transmit(&hspi2, tx, 2U, BMI088_SPI_TIMEOUT_MS);
    BMI088_Accel_Deselect();

    return status;
}

/**
  * @brief  Reads one gyroscope register using blocking SPI during initialization.
  * @param  reg Gyroscope register address.
  * @param  data Pointer to the received register value.
  * @retval HAL status.
  */
static HAL_StatusTypeDef BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx[2] = {0U};
    uint8_t rx[2] = {0U};

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    tx[0] = reg | BMI088_SPI_READ_MASK;

    BMI088_Gyro_Select();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, BMI088_SPI_TIMEOUT_MS);
    BMI088_Gyro_Deselect();

    if (status == HAL_OK)
    {
        *data = rx[1];
    }

    return status;
}

/**
  * @brief  Writes one gyroscope register using blocking SPI during initialization.
  * @param  reg Gyroscope register address.
  * @param  data Register value to write.
  * @retval HAL status.
  */
static HAL_StatusTypeDef BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx[2];

    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    tx[0] = reg & BMI088_SPI_WRITE_MASK;
    tx[1] = data;

    BMI088_Gyro_Select();
    status = HAL_SPI_Transmit(&hspi2, tx, 2U, BMI088_SPI_TIMEOUT_MS);
    BMI088_Gyro_Deselect();

    return status;
}

/**
  * @brief  Configures the BMI088 data-ready interrupt outputs used by STM32 EXTI.
  * @retval HAL status.
  */
static HAL_StatusTypeDef BMI088_DRDY_Config(void)
{
    HAL_StatusTypeDef status;

    /* Configure accelerometer INT1 as push-pull, active high and enable its output. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_INT1_IO_CTRL_REG, BMI088_ACC_INT1_ACTIVE_HIGH_PP);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Route accelerometer data-ready to INT1, connected to STM32 PC14. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_INT_MAP_DATA_REG, BMI088_ACC_DRDY_TO_INT1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Enable the gyroscope new-data interrupt source. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT_CTRL_REG, BMI088_GYRO_NEW_DATA_INT_ENABLE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Configure gyroscope INT3 as push-pull and active high. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_CONF_REG, BMI088_GYRO_INT3_ACTIVE_HIGH_PP);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Route gyroscope data-ready to INT3, connected to STM32 PC15. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_MAP_REG, BMI088_GYRO_DRDY_TO_INT3);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/* Exported functions --------------------------------------------------------*/

/*
 * BMI088 sensor configuration used by this flight stack:
 *
 * Accelerometer:
 *   ODR       : 800 Hz, sample period approximately 1.25 ms.
 *   Bandwidth : Normal bandwidth mode.
 *   Range     : ±24 g to preserve headroom during high-vibration and high-acceleration flight events.
 *   DRDY      : INT1, push-pull, active high, connected to STM32 PC14.
 *
 * Gyroscope:
 *   Mode      : Normal mode.
 *   ODR       : 1000 Hz, sample period 1 ms.
 *   Bandwidth : 116 Hz.
 *   Range     : ±2000 degree/s for quadrotor angular-rate measurement.
 *   DRDY      : INT3, push-pull, active high, connected to STM32 PC15.
 *
 * Communication:
 *   SPI2 is shared by the accelerometer and gyroscope with independent chip-select signals.
 *   Blocking SPI is used only while this initialization routine configures device registers.
 *   Runtime XYZ acquisition is performed exclusively by SPI2 DMA.
 */

/**
  * @brief  Initializes the BMI088 accelerometer, gyroscope and data-ready outputs.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t acc_chip_id = 0U;
    uint8_t gyro_chip_id = 0U;
    uint8_t dummy = 0U;

    /* Reset software state before accessing either BMI088 device. */
    bmi088_spi_state = BMI088_SPI_IDLE;
    bmi088_acc_dma_done = 0U;
    bmi088_gyro_dma_done = 0U;
    bmi088_acc_dma_error = 0U;
    bmi088_gyro_dma_error = 0U;

    /* Keep both devices deselected before the first SPI transaction. */
    BMI088_Accel_Deselect();
    BMI088_Gyro_Deselect();
    HAL_Delay(2U);

    /*
     * The accelerometer powers up in I2C mode. A CSB1 rising edge generated by
     * an initial SPI access switches the accelerometer interface to SPI mode.
     * The first returned byte is not used as the final CHIP_ID verification.
     */
    status = BMI088_Accel_ReadReg(BMI088_ACC_CHIP_ID_REG, &dummy);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Verify the accelerometer identity before applying configuration. */
    status = BMI088_Accel_ReadReg(BMI088_ACC_CHIP_ID_REG, &acc_chip_id);
    if (status != HAL_OK)
    {
        return status;
    }

    if (acc_chip_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    /* Verify the gyroscope identity before applying configuration. */
    status = BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID_REG, &gyro_chip_id);
    if (status != HAL_OK)
    {
        return status;
    }

    if (gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    /* Reset the accelerometer to a known device state. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    /*
     * Soft reset clears the accelerometer interface state, so perform another
     * dummy SPI register access before continuing with accelerometer setup.
     */
    status = BMI088_Accel_ReadReg(BMI088_ACC_CHIP_ID_REG, &dummy);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Leave accelerometer suspend mode and enter active power configuration. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_PWR_CONF_REG, BMI088_ACC_ACTIVE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(5U);

    /* Enable accelerometer measurement circuitry. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_ENABLE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(5U);

    /* Configure accelerometer normal bandwidth and 800 Hz output data rate. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_CONF_REG, BMI088_ACC_CONF_800HZ_NORMAL);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    /* Configure accelerometer full-scale range to ±24 g. */
    status = BMI088_Accel_WriteReg(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_24G);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    /* Reset the gyroscope to a known device state. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(30U);

    /* Place the gyroscope in normal operating mode. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(30U);

    /* Configure gyroscope full-scale range to ±2000 degree/s. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    /* Configure gyroscope output data rate to 1000 Hz with 116 Hz bandwidth. */
    status = BMI088_Gyro_WriteReg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_1000_116);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    /* Configure accelerometer and gyroscope data-ready interrupt outputs. */
    status = BMI088_DRDY_Config();
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
  * @brief  Starts an SPI2 DMA transaction for one accelerometer XYZ sample.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Accel_ReadRaw_DMA_Start(void)
{
    HAL_StatusTypeDef status;

    /* SPI2 may be owned by only one BMI088 DMA transaction at a time. */
    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    /* Do not overwrite an accelerometer sample that has not yet been consumed. */
    if (bmi088_acc_dma_done != 0U)
    {
        return HAL_BUSY;
    }

    /*
     * Prepare the burst-read command. Remaining TX bytes are dummy bytes that
     * generate SPI clocks while the accelerometer returns its XYZ registers.
     */
    bmi088_acc_dma_tx[0] = BMI088_ACC_X_LSB_REG | BMI088_SPI_READ_MASK;

    /*
     * Clean TX so DMA reads the latest command from RAM. Invalidate RX before
     * DMA because this dedicated 32-byte cache line will be written by DMA.
     */
    BMI088_DMA_CleanTxBuffer(bmi088_acc_dma_tx);
    BMI088_DMA_InvalidateRxBuffer(bmi088_acc_dma_rx);

    /* Claim SPI2 for the accelerometer before asserting chip-select. */
    bmi088_spi_state = BMI088_SPI_ACC_DMA;
    BMI088_Accel_Select();

    /*
     * Eight bytes are transferred: command, protocol dummy byte, then six XYZ
     * data bytes. Completion is finalized in BMI088_SPI_TxRxCpltHandler().
     */
    status = HAL_SPI_TransmitReceive_DMA(&hspi2, bmi088_acc_dma_tx, bmi088_acc_dma_rx, BMI088_ACC_DMA_TRANSFER_SIZE);

    if (status != HAL_OK)
    {
        BMI088_Accel_Deselect();
        bmi088_spi_state = BMI088_SPI_IDLE;
        bmi088_acc_dma_error = 1U;
    }

    return status;
}

/**
  * @brief  Gets the completed accelerometer DMA sample and clears its done flag.
  * @param  raw Pointer to the raw accelerometer output structure.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Accel_ReadRaw_DMA_Get(BMI088_RawData_t *raw)
{
    if (raw == NULL)
    {
        return HAL_ERROR;
    }

    if (bmi088_acc_dma_done == 0U)
    {
        return HAL_BUSY;
    }

    /*
     * DMA has updated RAM directly. Invalidate the RX cache line before the CPU
     * decodes the received bytes so stale cached data cannot be used.
     */
    BMI088_DMA_InvalidateRxBuffer(bmi088_acc_dma_rx);

    /*
     * Accelerometer RX layout:
     * rx[0] ignored, rx[1] protocol dummy,
     * rx[2:3] X, rx[4:5] Y, rx[6:7] Z, little-endian.
     */
    raw->x = (int16_t)(((uint16_t)bmi088_acc_dma_rx[3] << 8U) | (uint16_t)bmi088_acc_dma_rx[2]);
    raw->y = (int16_t)(((uint16_t)bmi088_acc_dma_rx[5] << 8U) | (uint16_t)bmi088_acc_dma_rx[4]);
    raw->z = (int16_t)(((uint16_t)bmi088_acc_dma_rx[7] << 8U) | (uint16_t)bmi088_acc_dma_rx[6]);

    /* Mark the sample as consumed so the next accelerometer DMA may start. */
    bmi088_acc_dma_done = 0U;

    return HAL_OK;
}

/**
  * @brief  Starts an SPI2 DMA transaction for one gyroscope XYZ sample.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Gyro_ReadRaw_DMA_Start(void)
{
    HAL_StatusTypeDef status;

    /* SPI2 may be owned by only one BMI088 DMA transaction at a time. */
    if (bmi088_spi_state != BMI088_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    /* Do not overwrite a gyroscope sample that has not yet been consumed. */
    if (bmi088_gyro_dma_done != 0U)
    {
        return HAL_BUSY;
    }

    /*
     * Prepare the burst-read command. Remaining TX bytes are dummy bytes that
     * generate SPI clocks while the gyroscope returns its XYZ registers.
     */
    bmi088_gyro_dma_tx[0] = BMI088_GYRO_RATE_X_LSB_REG | BMI088_SPI_READ_MASK;

    /*
     * Clean TX so DMA reads the latest command from RAM. Invalidate RX before
     * DMA because this dedicated 32-byte cache line will be written by DMA.
     */
    BMI088_DMA_CleanTxBuffer(bmi088_gyro_dma_tx);
    BMI088_DMA_InvalidateRxBuffer(bmi088_gyro_dma_rx);

    /* Claim SPI2 for the gyroscope before asserting chip-select. */
    bmi088_spi_state = BMI088_SPI_GYRO_DMA;
    BMI088_Gyro_Select();

    /*
     * Seven bytes are transferred: command followed by six XYZ data bytes.
     * Completion is finalized in BMI088_SPI_TxRxCpltHandler().
     */
    status = HAL_SPI_TransmitReceive_DMA(&hspi2, bmi088_gyro_dma_tx, bmi088_gyro_dma_rx, BMI088_GYRO_DMA_TRANSFER_SIZE);

    if (status != HAL_OK)
    {
        BMI088_Gyro_Deselect();
        bmi088_spi_state = BMI088_SPI_IDLE;
        bmi088_gyro_dma_error = 1U;
    }

    return status;
}

/**
  * @brief  Gets the completed gyroscope DMA sample and clears its done flag.
  * @param  raw Pointer to the raw gyroscope output structure.
  * @retval HAL status.
  */
HAL_StatusTypeDef BMI088_Gyro_ReadRaw_DMA_Get(BMI088_RawData_t *raw)
{
    if (raw == NULL)
    {
        return HAL_ERROR;
    }

    if (bmi088_gyro_dma_done == 0U)
    {
        return HAL_BUSY;
    }

    /*
     * DMA has updated RAM directly. Invalidate the RX cache line before the CPU
     * decodes the received bytes so stale cached data cannot be used.
     */
    BMI088_DMA_InvalidateRxBuffer(bmi088_gyro_dma_rx);

    /*
     * Gyroscope RX layout:
     * rx[0] ignored, rx[1:2] X, rx[3:4] Y, rx[5:6] Z, little-endian.
     */
    raw->x = (int16_t)(((uint16_t)bmi088_gyro_dma_rx[2] << 8U) | (uint16_t)bmi088_gyro_dma_rx[1]);
    raw->y = (int16_t)(((uint16_t)bmi088_gyro_dma_rx[4] << 8U) | (uint16_t)bmi088_gyro_dma_rx[3]);
    raw->z = (int16_t)(((uint16_t)bmi088_gyro_dma_rx[6] << 8U) | (uint16_t)bmi088_gyro_dma_rx[5]);

    /* Mark the sample as consumed so the next gyroscope DMA may start. */
    bmi088_gyro_dma_done = 0U;

    return HAL_OK;
}

/**
  * @brief  Returns whether the accelerometer DMA sample is ready to be consumed.
  * @retval 1 if a completed sample is available, otherwise 0.
  */
uint8_t BMI088_Accel_DMA_IsDone(void)
{
    return bmi088_acc_dma_done;
}

/**
  * @brief  Returns whether the gyroscope DMA sample is ready to be consumed.
  * @retval 1 if a completed sample is available, otherwise 0.
  */
uint8_t BMI088_Gyro_DMA_IsDone(void)
{
    return bmi088_gyro_dma_done;
}

/**
  * @brief  Returns and clears the accelerometer DMA error flag.
  * @retval Previous accelerometer DMA error flag value.
  */
uint8_t BMI088_Accel_DMA_GetAndClearError(void)
{
    uint8_t error = bmi088_acc_dma_error;
    bmi088_acc_dma_error = 0U;
    return error;
}

/**
  * @brief  Returns and clears the gyroscope DMA error flag.
  * @retval Previous gyroscope DMA error flag value.
  */
uint8_t BMI088_Gyro_DMA_GetAndClearError(void)
{
    uint8_t error = bmi088_gyro_dma_error;
    bmi088_gyro_dma_error = 0U;
    return error;
}

/**
  * @brief  Converts raw accelerometer data to g using the configured ±24 g range.
  * @param  raw Pointer to raw accelerometer data.
  * @param  accel_g Pointer to converted accelerometer data in g.
  */
void BMI088_Accel_ConvertToG(const BMI088_RawData_t *raw, BMI088_Data_t *accel_g)
{
    if ((raw == NULL) || (accel_g == NULL))
    {
        return;
    }

    accel_g->x = (float)raw->x * BMI088_ACC_SCALE_G;
    accel_g->y = (float)raw->y * BMI088_ACC_SCALE_G;
    accel_g->z = (float)raw->z * BMI088_ACC_SCALE_G;
}

/**
  * @brief  Converts raw gyroscope data to degree/s using the configured ±2000 dps range.
  * @param  raw Pointer to raw gyroscope data.
  * @param  gyro_dps Pointer to converted gyroscope data in degree/s.
  */
void BMI088_Gyro_ConvertToDps(const BMI088_RawData_t *raw, BMI088_Data_t *gyro_dps)
{
    if ((raw == NULL) || (gyro_dps == NULL))
    {
        return;
    }

    gyro_dps->x = (float)raw->x * BMI088_GYRO_SCALE_DPS;
    gyro_dps->y = (float)raw->y * BMI088_GYRO_SCALE_DPS;
    gyro_dps->z = (float)raw->z * BMI088_GYRO_SCALE_DPS;
}

/**
  * @brief  Returns whether SPI2 is currently owned by an active BMI088 DMA transfer.
  * @retval 1 if SPI2 is busy, otherwise 0.
  */
uint8_t BMI088_SPI_IsBusy(void)
{
    return (bmi088_spi_state != BMI088_SPI_IDLE) ? 1U : 0U;
}

/**
  * @brief  Returns the current BMI088 SPI2 state.
  * @retval Current BMI088 SPI state.
  */
BMI088_SPI_State_t BMI088_SPI_GetState(void)
{
    return bmi088_spi_state;
}

/**
  * @brief  Handles the HAL SPI transmit/receive DMA complete callback for BMI088.
  * @param  hspi Pointer to the SPI handle provided by HAL.
  */
void BMI088_SPI_TxRxCpltHandler(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi2)
    {
        return;
    }

    /*
     * Release the chip-select belonging to the completed transaction, publish
     * the corresponding sample-ready flag and return ownership of SPI2 to IDLE.
     */
    if (bmi088_spi_state == BMI088_SPI_ACC_DMA)
    {
        BMI088_Accel_Deselect();
        bmi088_spi_state = BMI088_SPI_IDLE;
        bmi088_acc_dma_done = 1U;
    }
    else if (bmi088_spi_state == BMI088_SPI_GYRO_DMA)
    {
        BMI088_Gyro_Deselect();
        bmi088_spi_state = BMI088_SPI_IDLE;
        bmi088_gyro_dma_done = 1U;
    }
}

/**
  * @brief  Handles the HAL SPI error callback for BMI088.
  * @param  hspi Pointer to the SPI handle provided by HAL.
  */
void BMI088_SPI_ErrorHandler(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi2)
    {
        return;
    }

    /*
     * Release the active chip-select and record the error for the sensor that
     * owned SPI2 when the HAL error occurred.
     */
    if (bmi088_spi_state == BMI088_SPI_ACC_DMA)
    {
        BMI088_Accel_Deselect();
        bmi088_acc_dma_done = 0U;
        bmi088_acc_dma_error = 1U;
    }
    else if (bmi088_spi_state == BMI088_SPI_GYRO_DMA)
    {
        BMI088_Gyro_Deselect();
        bmi088_gyro_dma_done = 0U;
        bmi088_gyro_dma_error = 1U;
    }
    else
    {
        BMI088_Accel_Deselect();
        BMI088_Gyro_Deselect();
    }

    bmi088_spi_state = BMI088_SPI_IDLE;
}
