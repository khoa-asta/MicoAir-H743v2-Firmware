/*
 * crsf.c
 *
 * CRSF byte-stream parser for an ELRS receiver.
 * Validates frame length and CRC before decoding 16 packed RC channels.
 * Run from the single stream owner; UART interrupts only transport bytes.
 *
 * Author: Viết Khoa
 */

/* Includes ------------------------------------------------------------------*/
#include "crsf.h"
#include <stddef.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define CRSF_FRAME_PREFIX_SIZE                   2U
#define CRSF_FRAME_LENGTH_MIN                    2U
#define CRSF_FRAME_LENGTH_MAX                    (CRSF_MAX_FRAME_SIZE - CRSF_FRAME_PREFIX_SIZE)
#define CRSF_RC_PAYLOAD_SIZE                     22U
#define CRSF_RC_LENGTH_MIN                       (CRSF_RC_PAYLOAD_SIZE + 2U)
#define CRSF_CHANNEL_BITS                        11U
#define CRSF_CHANNEL_MASK                        0x07FFU

/* Private constants ---------------------------------------------------------*/
/* Four CRC-8/DVB-S2 shifts per lookup; two lookups replace eight bit steps.
 * Polynomial 0xD5, initial value 0. Table occupies 16 bytes of read-only data.
 */
static const uint8_t crsf_crc_nibble[16] =
{
    0x00U, 0xD5U, 0x7FU, 0xAAU, 0xFEU, 0x2BU, 0x81U, 0x54U,
    0x29U, 0xFCU, 0x56U, 0x83U, 0xD7U, 0x02U, 0xA8U, 0x7DU
};

/* Private variables ---------------------------------------------------------*/
static uint8_t frame_buffer[CRSF_MAX_FRAME_SIZE];
static uint8_t buffered;
static uint8_t have_channels;
static uint16_t channels[CRSF_CHANNEL_COUNT];
static CRSF_Stats_t stats;

/* Private function prototypes -----------------------------------------------*/
static uint8_t CRSF_IsSync(uint8_t value);
static uint8_t CRSF_FrameCRC(const uint8_t *data, uint8_t length);
static void CRSF_DropPrefix(uint8_t count);
static void CRSF_DecodeChannels(const uint8_t *payload);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Recognizes serial sync and supported TBS device addresses.
  * @note   Accepts broadcast and dynamic addresses; no routing is performed.
  */
static uint8_t CRSF_IsSync(uint8_t value)
{
    if ((value >= 0x20U) && (value <= 0x7FU))
    {
        return 1U;
    }

    switch (value)
    {
        case 0x00U: case 0x0EU: case 0x10U: case 0x12U: case 0x13U:
        case 0x14U: case 0x80U: case 0x8AU:
        case 0x90U: case 0x91U: case 0x92U: case 0x93U:
        case 0x94U: case 0x95U: case 0x96U: case 0x97U:
        case 0xB0U: case 0xB2U: case 0xC0U: case 0xC2U: case 0xC4U:
        case 0xC8U: case 0xCAU: case 0xCCU: case 0xCEU:
        case 0xEAU: case 0xEBU: case 0xECU: case 0xEDU: case 0xEEU:
        case 0xF0U: case 0xF2U:
            return 1U;

        default:
            return 0U;
    }
}

/**
  * @brief  Computes CRC over frame type and payload, excluding sync/length.
  */
static uint8_t CRSF_FrameCRC(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    for (uint8_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        crc = (uint8_t)((uint8_t)(crc << 4U) ^ crsf_crc_nibble[crc >> 4U]);
        crc = (uint8_t)((uint8_t)(crc << 4U) ^ crsf_crc_nibble[crc >> 4U]);
    }

    return crc;
}

/**
  * @brief  Removes a consumed prefix while preserving a possible next frame.
  * @param  count Number of buffered bytes to remove; must not exceed buffered.
  */
static void CRSF_DropPrefix(uint8_t count)
{
    buffered = (uint8_t)(buffered - count);
    if (buffered != 0U)
    {
        memmove(frame_buffer, &frame_buffer[count], buffered);
    }
}

/**
  * @brief  Unpacks 22 bytes into 16 raw channels without C bitfield assumptions.
  * @param  payload CRC-validated payload containing at least 22 bytes.
  */
static void CRSF_DecodeChannels(const uint8_t *payload)
{
    uint32_t bits = 0U;
    uint8_t bit_count = 0U;
    uint8_t position = 0U;

    for (uint8_t channel = 0U; channel < CRSF_CHANNEL_COUNT; ++channel)
    {
        while (bit_count < CRSF_CHANNEL_BITS)
        {
            bits |= (uint32_t)payload[position++] << bit_count;
            bit_count = (uint8_t)(bit_count + 8U);
        }

        channels[channel] = (uint16_t)(bits & CRSF_CHANNEL_MASK);
        bits >>= CRSF_CHANNEL_BITS;
        bit_count = (uint8_t)(bit_count - CRSF_CHANNEL_BITS);
    }

    have_channels = 1U;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Clears stream state, statistics and the last decoded channels.
  */
void CRSF_Init(void)
{
    buffered = 0U;
    have_channels = 0U;
    memset(channels, 0, sizeof(channels));
    memset(&stats, 0, sizeof(stats));
}

/**
  * @brief  Appends a byte and consumes all complete frame candidates.
  * @retval Number of newly decoded RC frames; only the latest is retained.
  */
uint8_t CRSF_ProcessByte(uint8_t byte)
{
    uint8_t updates = 0U;

    stats.bytes_in++;
    /* Complete candidates are consumed or shortened below. At entry there
     * are at most 63 bytes, so the append stays within this 64-byte array.
     */
    frame_buffer[buffered++] = byte;

    while (buffered != 0U)
    {
        uint8_t length;
        uint8_t total;

        if (CRSF_IsSync(frame_buffer[0]) == 0U)
        {
            stats.discarded_bytes++;
            CRSF_DropPrefix(1U);
            continue;
        }
        if (buffered < CRSF_FRAME_PREFIX_SIZE)
        {
            break;
        }

        length = frame_buffer[1];
        if ((length < CRSF_FRAME_LENGTH_MIN) || (length > CRSF_FRAME_LENGTH_MAX))
        {
            stats.length_errors++;
            stats.discarded_bytes++;
            CRSF_DropPrefix(1U);
            continue;
        }

        total = (uint8_t)(length + CRSF_FRAME_PREFIX_SIZE);
        if (buffered < total)
        {
            break;
        }
        if (CRSF_FrameCRC(&frame_buffer[2], (uint8_t)(length - 1U)) !=
            frame_buffer[total - 1U])
        {
            stats.crc_errors++;
            stats.discarded_bytes++;
            /* Slide one byte after a bad candidate; retain possible syncs.
             * A plausible but incomplete candidate needs more input or a
             * caller-confirmed frame timeout followed by ResetStream().
             */
            CRSF_DropPrefix(1U);
            continue;
        }

        stats.valid_frames++;
        if (frame_buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            if (length >= CRSF_RC_LENGTH_MIN)
            {
                /* Allow trailing fields for protocol compatibility, but
                 * verify the CRC over the entire declared frame first.
                 */
                CRSF_DecodeChannels(&frame_buffer[3]);
                stats.rc_frames++;
                updates++;
            }
            else
            {
                stats.short_rc_frames++;
            }
        }
        else
        {
            stats.other_frames++;
        }

        CRSF_DropPrefix(total);
    }

    return updates;
}

/**
  * @brief  Copies historical raw channels; does not assess RC link freshness.
  */
uint8_t CRSF_GetChannels(uint16_t out[CRSF_CHANNEL_COUNT])
{
    if ((out == NULL) || (have_channels == 0U))
    {
        return 0U;
    }

    memcpy(out, channels, sizeof(channels));
    return 1U;
}

/**
  * @brief  Copies the parser statistics for the owning task.
  */
void CRSF_GetStats(CRSF_Stats_t *out)
{
    if (out != NULL)
    {
        *out = stats;
    }
}

/**
  * @brief  Discards a partial stream while preserving historical channels.
  */
void CRSF_ResetStream(void)
{
    stats.discarded_bytes += buffered;
    buffered = 0U;
    stats.stream_resets++;
}
