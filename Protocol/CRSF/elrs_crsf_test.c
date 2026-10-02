/*
 * elrs_crsf_test.c
 *
 * CRSF checkpoint over the USART6 ELRS loopback transport.
 * Exercises complete/split frames, bursts, CRC rejection and stream recovery.
 * The fixed frame and independent test CRC do not call parser internals.
 *
 * Author: Viết Khoa
 */

/* Includes ------------------------------------------------------------------*/
#include "elrs_crsf_test.h"
#include "elrs.h"
#include "crsf.h"
#include "usart.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define TEST_TIMEOUT_MS 100U
#define TEST_GAP_MS 2U

/* Private constants ---------------------------------------------------------*/
/* Literal fixture generated with independent integer packing and polynomial
 * division. No production parser helper is used to build this frame. */
static const uint8_t good_frame[26] = {
    0xC8U, 0x18U, 0x16U, 0xACU, 0x00U, 0xDFU, 0xC4U, 0x59U, 0x01U,
    0xBEU, 0x89U, 0xB3U, 0x02U, 0x7CU, 0x13U, 0x67U, 0x05U, 0xF8U,
    0x26U, 0xCEU, 0x0AU, 0xF0U, 0x4DU, 0x1CU, 0x7CU, 0xE0U
};
static const uint16_t expected_channels[16] = {
    172U, 992U, 1811U, 172U, 992U, 1811U, 172U, 992U,
    1811U, 172U, 992U, 1811U, 172U, 992U, 1811U, 992U
};
/* {RC frames, CRC-valid frames, CRC errors, length errors, other, short RC}. */
static const uint32_t expected_stats[8][6] = {
    {1U, 1U, 0U, 0U, 0U, 0U}, {2U, 2U, 0U, 0U, 0U, 0U},
    {5U, 5U, 0U, 0U, 0U, 0U}, {5U, 5U, 1U, 0U, 0U, 0U},
    {6U, 6U, 1U, 2U, 0U, 0U}, {6U, 7U, 1U, 2U, 1U, 0U},
    {6U, 8U, 1U, 2U, 1U, 1U}, {7U, 9U, 1U, 2U, 1U, 1U}
};

/* Debug variables -----------------------------------------------------------*/
/* Definitions belong to this file; keep only extern declarations elsewhere. */
volatile uint32_t dbg_crsf_test_done;
volatile uint32_t dbg_crsf_test_pass;
volatile uint32_t dbg_crsf_test_phase;
volatile uint32_t dbg_crsf_test_phases_passed;
volatile uint32_t dbg_crsf_test_failure_code;
volatile uint32_t dbg_crsf_test_tx_bytes;
volatile uint32_t dbg_crsf_test_rx_bytes;
volatile uint32_t dbg_crsf_test_mismatch_count;
volatile uint32_t dbg_crsf_test_channel_mismatch_count;
volatile uint32_t dbg_crsf_test_timeout_count;
volatile uint32_t dbg_crsf_test_extra_bytes;
volatile uint32_t dbg_crsf_test_rc_frames;
volatile uint32_t dbg_crsf_test_valid_frames;
volatile uint32_t dbg_crsf_test_crc_errors;
volatile uint32_t dbg_crsf_test_length_errors;
volatile uint32_t dbg_crsf_test_other_frames;
volatile uint32_t dbg_crsf_test_short_rc_frames;
volatile uint16_t dbg_crsf_test_channels[16];
volatile HAL_StatusTypeDef dbg_crsf_test_result = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_crsf_test_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_crsf_test_start_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_crsf_test_tx_status = HAL_ERROR;
volatile HAL_StatusTypeDef dbg_crsf_test_stop_status = HAL_ERROR;

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[78]; /* Polling TX: ordinary CPU RAM is sufficient. */

_Static_assert(CRSF_CHANNEL_COUNT == 16U, "The fixture contains 16 RC channels");
_Static_assert(sizeof(tx_buffer) <= ELRS_RX_FIFO_SIZE,
               "The FIFO must hold the longest polling-TX burst");

/* Private function prototypes -----------------------------------------------*/
static uint8_t CRSF_TestCRC(const uint8_t *data, uint16_t length);
static void CRSF_TestSnapshotStats(void);
static uint8_t CRSF_TestCheckChannels(void);
static HAL_StatusTypeDef CRSF_TestExchange(uint8_t *data, uint16_t length);
static uint8_t CRSF_TestCheckPhase(uint32_t phase);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Computes fixture CRC with independent polynomial division.
  */
static uint8_t CRSF_TestCRC(const uint8_t *data, uint16_t length)
{
    uint32_t remainder = 0U;
    for (uint16_t i = 0U; i < length; ++i)
    {
        remainder ^= (uint32_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t top = remainder & 0x8000U;
            remainder = (remainder << 1U) & 0xFFFFU;
            if (top != 0U) { remainder ^= 0xD500U; }
        }
    }
    return (uint8_t)(remainder >> 8U);
}

/**
  * @brief  Copies parser counters to the existing checkpoint debug variables.
  */
static void CRSF_TestSnapshotStats(void)
{
    CRSF_Stats_t s;
    CRSF_GetStats(&s);
    dbg_crsf_test_rc_frames = s.rc_frames;
    dbg_crsf_test_valid_frames = s.valid_frames;
    dbg_crsf_test_crc_errors = s.crc_errors;
    dbg_crsf_test_length_errors = s.length_errors;
    dbg_crsf_test_other_frames = s.other_frames;
    dbg_crsf_test_short_rc_frames = s.short_rc_frames;
}

/**
  * @brief  Compares the decoded snapshot with all 16 fixture channels.
  */
static uint8_t CRSF_TestCheckChannels(void)
{
    uint16_t actual[16];
    if (CRSF_GetChannels(actual) == 0U) { return 0U; }
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        dbg_crsf_test_channels[i] = actual[i];
        if (actual[i] != expected_channels[i])
        {
            dbg_crsf_test_channel_mismatch_count++;
        }
    }
    return (dbg_crsf_test_channel_mismatch_count == 0U) ? 1U : 0U;
}

/**
  * @brief  Sends a burst, verifies echoed bytes and feeds CRSF in seven-byte reads.
  * @note   Preserves parser state across UART IDLE and separate exchanges.
  */
static HAL_StatusTypeDef CRSF_TestExchange(uint8_t *data, uint16_t length)
{
    uint8_t received[7]; /* Deliberately unrelated to CRSF frame boundaries. */
    uint16_t offset = 0U;
    uint32_t started;
    /* Mutable CPU buffer supports HAL versions with a non-const TX pointer. */
    dbg_crsf_test_tx_status = HAL_UART_Transmit(&huart6, data,
                                               length, TEST_TIMEOUT_MS);
    if (dbg_crsf_test_tx_status != HAL_OK)
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_TX;
        return dbg_crsf_test_tx_status;
    }
    dbg_crsf_test_tx_bytes += length;
    started = HAL_GetTick();
    while (offset < length)
    {
        uint16_t n;
        if (ELRS_HasRxFault() != 0U)
        {
            dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_RX;
            return HAL_ERROR;
        }
        n = ELRS_Read(received, sizeof(received));
        if (ELRS_HasRxFault() != 0U)
        {
            dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_RX;
            return HAL_ERROR;
        }
        for (uint16_t i = 0U; i < n; ++i)
        {
            dbg_crsf_test_rx_bytes++;
            if (offset >= length)
            {
                dbg_crsf_test_extra_bytes++;
                dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_EXTRA;
                return HAL_ERROR;
            }
            if (received[i] != data[offset++])
            {
                dbg_crsf_test_mismatch_count++;
                dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_DATA;
                return HAL_ERROR;
            }
            if ((CRSF_ProcessByte(received[i]) != 0U) && (CRSF_TestCheckChannels() == 0U))
            {
                dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_CHANNELS;
                return HAL_ERROR;
            }
        }
        if (ELRS_HasRxFault() != 0U)
        {
            dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_RX;
            return HAL_ERROR;
        }
        if ((offset < length) && ((HAL_GetTick() - started) >= TEST_TIMEOUT_MS))
        {
            dbg_crsf_test_timeout_count++;
            dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_TIMEOUT;
            return HAL_TIMEOUT;
        }
    }
    HAL_Delay(TEST_GAP_MS);
    if (ELRS_HasRxFault() != 0U)
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_RX;
        return HAL_ERROR;
    }
    if (ELRS_Available() != 0U)
    {
        dbg_crsf_test_extra_bytes += ELRS_Available();
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_EXTRA;
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  Verifies cumulative parser counters for a completed phase.
  */
static uint8_t CRSF_TestCheckPhase(uint32_t phase)
{
    const uint32_t *e = expected_stats[phase - 1U];
    CRSF_TestSnapshotStats();
    return ((dbg_crsf_test_rc_frames == e[0]) &&
            (dbg_crsf_test_valid_frames == e[1]) &&
            (dbg_crsf_test_crc_errors == e[2]) &&
            (dbg_crsf_test_length_errors == e[3]) &&
            (dbg_crsf_test_other_frames == e[4]) &&
            (dbg_crsf_test_short_rc_frames == e[5])) ? 1U : 0U;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Runs eight UART/CRSF checkpoint phases under exclusive bench ownership.
  * @retval HAL status; phase and failure_code identify the failing checkpoint.
  */
HAL_StatusTypeDef ELRS_CRSFTest_Run(void)
{
    HAL_StatusTypeDef result = HAL_ERROR;
    uint8_t owns_rx = 0U;
    uint8_t owns_parser = 0U;
    dbg_crsf_test_done = 0U;
    dbg_crsf_test_pass = 0U;
    dbg_crsf_test_phase = 0U;
    dbg_crsf_test_phases_passed = 0U;
    dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_NONE;
    dbg_crsf_test_tx_bytes = 0U;
    dbg_crsf_test_rx_bytes = 0U;
    dbg_crsf_test_mismatch_count = 0U;
    dbg_crsf_test_channel_mismatch_count = 0U;
    dbg_crsf_test_timeout_count = 0U;
    dbg_crsf_test_extra_bytes = 0U;
    dbg_crsf_test_result = HAL_ERROR;
    dbg_crsf_test_init_status = HAL_ERROR;
    dbg_crsf_test_start_status = HAL_ERROR;
    dbg_crsf_test_tx_status = HAL_ERROR;
    dbg_crsf_test_stop_status = HAL_ERROR;
    for (uint8_t i = 0U; i < 16U; ++i) { dbg_crsf_test_channels[i] = 0U; }
    dbg_crsf_test_rc_frames = 0U;
    dbg_crsf_test_valid_frames = 0U;
    dbg_crsf_test_crc_errors = 0U;
    dbg_crsf_test_length_errors = 0U;
    dbg_crsf_test_other_frames = 0U;
    dbg_crsf_test_short_rc_frames = 0U;

    dbg_crsf_test_init_status = ELRS_Init();
    if (dbg_crsf_test_init_status != HAL_OK)
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_INIT;
        result = dbg_crsf_test_init_status;
        goto finish;
    }
    /* Do not reset a live parser when ELRS_Init() rejects an active receiver. */
    CRSF_Init();
    owns_parser = 1U;
    CRSF_TestSnapshotStats();
    dbg_crsf_test_start_status = ELRS_Receive_DMA_Start();
    if (dbg_crsf_test_start_status != HAL_OK)
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_START;
        result = dbg_crsf_test_start_status;
        goto finish;
    }
    owns_rx = 1U;
    for (uint32_t phase = 1U; phase <= 8U; ++phase)
    {
        uint16_t length = 26U;
        dbg_crsf_test_phase = phase;
        memcpy(tx_buffer, good_frame, sizeof(good_frame));
        switch (phase)
        {
            case 2U:
                result = CRSF_TestExchange(tx_buffer, 3U);
                if (result != HAL_OK) { goto finish; }
                if (CRSF_TestCheckPhase(1U) == 0U) { goto counters_failed; }
                result = CRSF_TestExchange(&tx_buffer[3], 7U);
                if (result != HAL_OK) { goto finish; }
                if (CRSF_TestCheckPhase(1U) == 0U) { goto counters_failed; }
                result = CRSF_TestExchange(&tx_buffer[10], 16U);
                break;
            case 3U:
                memcpy(&tx_buffer[26], good_frame, 26U);
                memcpy(&tx_buffer[52], good_frame, 26U);
                length = 78U;
                break;
            case 4U:
                memset(&tx_buffer[3], 0xFF, 22U);
                tx_buffer[25] = 0xFEU; /* Deliberately wrong CRC. */
                break;
            case 5U:
            {
                static const uint8_t noise[] = {0xFFU, 0xFEU, 0xFDU,
                                               0xC8U, 1U, 0xC8U, 0xFFU};
                memcpy(tx_buffer, noise, sizeof(noise));
                memcpy(&tx_buffer[sizeof(noise)], good_frame, 26U);
                length = 33U;
                break;
            }
            case 6U:
                memset(tx_buffer, 0, 14U);
                tx_buffer[0] = 0xC8U;
                tx_buffer[1] = 12U;
                tx_buffer[2] = 0x14U; /* Valid Link Statistics, not RC channels. */
                tx_buffer[13] = CRSF_TestCRC(&tx_buffer[2], 11U);
                length = 14U;
                break;
            case 7U:
                tx_buffer[0] = 0xC8U;
                tx_buffer[1] = 3U;
                tx_buffer[2] = 0x16U;
                tx_buffer[3] = 1U; /* Too short for 16 channels, CRC correct. */
                tx_buffer[4] = CRSF_TestCRC(&tx_buffer[2], 2U);
                length = 5U;
                break;
            default:
                break;
        }
        if (phase != 2U) { result = CRSF_TestExchange(tx_buffer, length); }
        if (result != HAL_OK) { goto finish; }
        if (CRSF_TestCheckPhase(phase) == 0U) { goto counters_failed; }
        dbg_crsf_test_phases_passed++;
    }
    result = HAL_OK;
    goto finish;

counters_failed:
    dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_COUNTERS;
    result = HAL_ERROR;
finish:
    if (owns_rx != 0U)
    {
        dbg_crsf_test_stop_status = ELRS_Receive_DMA_Stop();
        if ((dbg_crsf_test_stop_status != HAL_OK) && (result == HAL_OK))
        {
            dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_STOP;
            result = dbg_crsf_test_stop_status;
        }
    }
    if ((result == HAL_OK) && (ELRS_HasRxFault() != 0U))
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_RX;
        result = HAL_ERROR;
    }
    if ((result == HAL_OK) && (ELRS_Available() != 0U))
    {
        dbg_crsf_test_failure_code = CRSF_TEST_FAILURE_EXTRA;
        dbg_crsf_test_extra_bytes += ELRS_Available();
        result = HAL_ERROR;
    }
    if (owns_parser != 0U)
    {
        CRSF_TestSnapshotStats();
    }
    dbg_crsf_test_result = result;
    dbg_crsf_test_pass = (result == HAL_OK) ? 1U : 0U;
    __DMB();
    dbg_crsf_test_done = 1U;
    return result;
}
