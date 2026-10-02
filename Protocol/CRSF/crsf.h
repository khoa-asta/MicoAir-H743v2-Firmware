/*
 * crsf.h
 *
 * CRSF byte-stream parser and raw RC channel interface.
 * Author: Viết Khoa
 */

#ifndef CRSF_CRSF_H_
#define CRSF_CRSF_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported defines ----------------------------------------------------------*/
#define CRSF_CHANNEL_COUNT                       16U
#define CRSF_MAX_FRAME_SIZE                      64U
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED         0x16U

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t bytes_in;
    uint32_t valid_frames;
    uint32_t rc_frames;
    uint32_t other_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t short_rc_frames;
    uint32_t discarded_bytes;
    uint32_t stream_resets;
} CRSF_Stats_t;

/* Exported functions --------------------------------------------------------*/
/* One serialized owner: main during bring-up, then RCTask. No ISR calls.
 * This module has no HAL/RTOS dependency and performs no dynamic allocation.
 * Error counters describe parser candidates, not physical RF packet losses.
 */

/**
  * @brief  Clears the parser, counters and historical channel snapshot.
  */
void CRSF_Init(void);

/**
  * @brief  Consumes one byte from the UART stream.
  * @param  byte Next received byte, in wire order.
  * @retval Number of RC frames decoded; resynchronization may release several.
  */
uint8_t CRSF_ProcessByte(uint8_t byte);

/**
  * @brief  Copies the latest decoded RC channels.
  * @param  out Destination for 16 unsigned 11-bit raw channel values.
  * @retval 1 if a snapshot exists; 0 for NULL or before the first RC frame.
  * @note   A historical snapshot does not indicate link freshness or failsafe.
  */
uint8_t CRSF_GetChannels(uint16_t out[CRSF_CHANNEL_COUNT]);

/**
  * @brief  Copies parser counters; a NULL destination is ignored.
  */
void CRSF_GetStats(CRSF_Stats_t *out);

/**
  * @brief  Drops partial input after a transport fault or confirmed frame timeout.
  * @note   Keeps counters and the last channels. Do not reset on every UART IDLE
  *         event or ELRS_Read() boundary; a frame may span several reads.
  */
void CRSF_ResetStream(void);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_CRSF_H_ */
