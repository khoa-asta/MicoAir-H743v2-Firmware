/*
 * elrs_test.c
 *
 * USART6 loopback checkpoint for the ELRS circular-DMA transport.
 * Polling TX sends deterministic bytes; RX DMA/IRQs feed the driver's FIFO.
 * This bench owns USART6 until cleanup and performs no CRSF decoding.
 *
 * Author: Viết Khoa
 */

/* Includes ------------------------------------------------------------------*/
#include "elrs_test.h"
#include "elrs.h"
#include "usart.h"

/* Private defines -----------------------------------------------------------*/
#define TEST_ROUNDS          4U
#define TEST_TX_TIMEOUT_MS   100U
#define TEST_RX_TIMEOUT_MS   100U
#define TEST_GAP_MS          2U

/* Private constants ---------------------------------------------------------*/
static const uint16_t chunk_sizes[] = {1U, 26U, 255U, 256U, 257U, 600U};
/* Private variables ---------------------------------------------------------*/
/* CPU-only buffers: polling TX does not require TX DMA/cache maintenance. */
static uint8_t tx_buffer[600];
static uint8_t rx_buffer[64];

_Static_assert(sizeof(tx_buffer) <= ELRS_RX_FIFO_SIZE,
               "The FIFO must hold the longest polling-TX burst");

/* Debug variables -----------------------------------------------------------*/
/* Definitions belong to this file; keep only extern declarations elsewhere. */
volatile uint32_t dbg_elrs_test_failure_code;
volatile uint32_t dbg_elrs_test_done;
volatile uint32_t dbg_elrs_test_pass;
volatile uint32_t dbg_elrs_test_expected_bytes;
volatile uint32_t dbg_elrs_test_tx_bytes;
volatile uint32_t dbg_elrs_test_rx_bytes;
volatile uint32_t dbg_elrs_test_chunks_passed;
volatile uint32_t dbg_elrs_test_mismatch_count;
volatile uint32_t dbg_elrs_test_timeout_count;
volatile uint32_t dbg_elrs_test_extra_byte_count;
volatile uint32_t dbg_elrs_test_first_bad_index;
volatile uint32_t dbg_elrs_test_first_expected;
volatile uint32_t dbg_elrs_test_first_actual;
volatile HAL_StatusTypeDef dbg_elrs_test_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_elrs_test_start_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_elrs_test_tx_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_elrs_test_stop_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_elrs_test_result = HAL_ERROR;

/* Private function prototypes -----------------------------------------------*/
static uint8_t ELRS_TestPattern(uint32_t index);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Generates a deterministic byte for an absolute stream position.
  */
static uint8_t ELRS_TestPattern(uint32_t index)
{
    return (uint8_t)(((index * 73U) ^ (index >> 8U) ^ 0xA5U) & 0xFFU);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Verifies byte order across short bursts, DMA wraps and FIFO reads.
  * @retval HAL status; dbg_elrs_test_failure_code identifies the failed step.
  */
HAL_StatusTypeDef ELRS_LoopbackTest_Run(void)
{
    HAL_StatusTypeDef result = HAL_ERROR;
    uint32_t stream_index = 0U;
    uint8_t owns_rx = 0U;

    dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_NONE;
    dbg_elrs_test_done = 0U;
    dbg_elrs_test_pass = 0U;
    dbg_elrs_test_expected_bytes = 0U;
    dbg_elrs_test_tx_bytes = 0U;
    dbg_elrs_test_rx_bytes = 0U;
    dbg_elrs_test_chunks_passed = 0U;
    dbg_elrs_test_mismatch_count = 0U;
    dbg_elrs_test_timeout_count = 0U;
    dbg_elrs_test_extra_byte_count = 0U;
    dbg_elrs_test_first_bad_index = UINT32_MAX;
    dbg_elrs_test_first_expected = 0U;
    dbg_elrs_test_first_actual = 0U;
    dbg_elrs_test_init_status = HAL_ERROR;
    dbg_elrs_test_start_status = HAL_ERROR;
    dbg_elrs_test_tx_status = HAL_ERROR;
    dbg_elrs_test_stop_status = HAL_ERROR;
    dbg_elrs_test_result = HAL_ERROR;

    for (uint32_t i = 0U; i < sizeof(chunk_sizes) / sizeof(chunk_sizes[0]); i++)
    {
        dbg_elrs_test_expected_bytes += TEST_ROUNDS * chunk_sizes[i];
    }

    dbg_elrs_test_init_status = ELRS_Init();
    if (dbg_elrs_test_init_status != HAL_OK)
    {
        dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_INIT;
        result = dbg_elrs_test_init_status;
        goto finish;
    }
    dbg_elrs_test_start_status = ELRS_Receive_DMA_Start();
    if (dbg_elrs_test_start_status != HAL_OK)
    {
        dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_START;
        result = dbg_elrs_test_start_status;
        goto finish;
    }
    owns_rx = 1U;

    for (uint32_t round = 0U; round < TEST_ROUNDS; round++)
    {
        for (uint32_t c = 0U; c < sizeof(chunk_sizes) / sizeof(chunk_sizes[0]); c++)
        {
            uint16_t length = chunk_sizes[c];
            uint16_t received = 0U;
            uint32_t started;

            for (uint16_t i = 0U; i < length; i++)
            {
                tx_buffer[i] = ELRS_TestPattern(stream_index + i);
            }

            /* Polling TX; RX DMA and its interrupts run during this call. */
            dbg_elrs_test_tx_status = HAL_UART_Transmit(
                &huart6, tx_buffer, length, TEST_TX_TIMEOUT_MS);
            if (dbg_elrs_test_tx_status != HAL_OK)
            {
                dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_TX;
                result = dbg_elrs_test_tx_status;
                goto finish;
            }
            dbg_elrs_test_tx_bytes += length;
            started = HAL_GetTick();

            while (received < length)
            {
                uint16_t capacity = (uint16_t)(length - received);
                uint16_t count;
                if (capacity > sizeof(rx_buffer))
                {
                    capacity = sizeof(rx_buffer);
                }

                if (ELRS_HasRxFault() != 0U)
                {
                    dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_RX;
                    goto finish;
                }
                count = ELRS_Read(rx_buffer, capacity);
                for (uint16_t i = 0U; i < count; i++)
                {
                    uint32_t index = stream_index + received + i;
                    uint8_t expected = ELRS_TestPattern(index);
                    if (rx_buffer[i] != expected)
                    {
                        if (dbg_elrs_test_mismatch_count == 0U)
                        {
                            dbg_elrs_test_first_bad_index = index;
                            dbg_elrs_test_first_expected = expected;
                            dbg_elrs_test_first_actual = rx_buffer[i];
                        }
                        dbg_elrs_test_mismatch_count++;
                    }
                }
                received = (uint16_t)(received + count);
                dbg_elrs_test_rx_bytes += count;

                if (ELRS_HasRxFault() != 0U)
                {
                    dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_RX;
                    goto finish;
                }
                if (dbg_elrs_test_mismatch_count != 0U)
                {
                    dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_DATA;
                    goto finish;
                }
                if ((received < length) &&
                    ((uint32_t)(HAL_GetTick() - started) >= TEST_RX_TIMEOUT_MS))
                {
                    dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_TIMEOUT;
                    dbg_elrs_test_timeout_count++;
                    result = HAL_TIMEOUT;
                    goto finish;
                }
            }

            /* Allow IDLE and any pending callback to run before the next burst. */
            HAL_Delay(TEST_GAP_MS);
            if (ELRS_HasRxFault() != 0U)
            {
                dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_RX;
                goto finish;
            }
            if (ELRS_Available() != 0U)
            {
                dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_EXTRA;
                dbg_elrs_test_extra_byte_count = ELRS_Available();
                goto finish;
            }

            stream_index += length;
            dbg_elrs_test_chunks_passed++;
        }
    }

    if ((dbg_elrs_test_tx_bytes == dbg_elrs_test_expected_bytes) &&
        (dbg_elrs_test_rx_bytes == dbg_elrs_test_expected_bytes))
    {
        result = HAL_OK;
    }
    else
    {
        dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_TOTAL;
    }

finish:
    if (owns_rx != 0U)
    {
        dbg_elrs_test_stop_status = ELRS_Receive_DMA_Stop();
        if ((dbg_elrs_test_stop_status != HAL_OK) && (result == HAL_OK))
        {
            dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_STOP;
            result = dbg_elrs_test_stop_status;
        }
    }
    /* Include an error latched by an IRQ just before reception was stopped. */
    if ((result == HAL_OK) && (ELRS_HasRxFault() != 0U))
    {
        dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_RX;
        result = HAL_ERROR;
    }
    if ((result == HAL_OK) && (ELRS_Available() != 0U))
    {
        dbg_elrs_test_failure_code = ELRS_TEST_FAILURE_EXTRA;
        dbg_elrs_test_extra_byte_count += ELRS_Available();
        result = HAL_ERROR;
    }
    dbg_elrs_test_result = result;
    dbg_elrs_test_pass = (result == HAL_OK) ? 1U : 0U;
    __DMB();
    dbg_elrs_test_done = 1U;
    return result;
}
