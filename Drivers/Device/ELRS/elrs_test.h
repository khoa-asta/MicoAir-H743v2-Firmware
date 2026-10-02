/*
 * elrs_test.h
 *
 * USART6 TX-to-RX loopback checkpoint for the ELRS transport.
 * Author: Viết Khoa
 */

#ifndef DEVICE_ELRS_ELRS_TEST_H_
#define DEVICE_ELRS_ELRS_TEST_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    ELRS_TEST_FAILURE_NONE = 0,
    ELRS_TEST_FAILURE_INIT = 1,
    ELRS_TEST_FAILURE_START = 2,
    ELRS_TEST_FAILURE_TX = 3,
    ELRS_TEST_FAILURE_TIMEOUT = 4,
    ELRS_TEST_FAILURE_DATA = 5,
    ELRS_TEST_FAILURE_RX = 6,
    ELRS_TEST_FAILURE_EXTRA = 7,
    ELRS_TEST_FAILURE_TOTAL = 8,
    ELRS_TEST_FAILURE_STOP = 9
} ELRS_TestFailure_t;

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Runs 24 polling-TX bursts through USART6 circular RX DMA.
  * @retval HAL_OK on pass; HAL_ERROR, HAL_BUSY or HAL_TIMEOUT on failure.
  * @note   Bench only: connect PC6/TX to PC7/RX and disconnect the receiver.
  *         Call after MPU/SRAM1/UART/DMA initialization, with HAL tick and IRQs
  *         running. Own USART6 exclusively; never run beside RCTask or in ISR.
  *         The test calls ELRS_Init/Start/Stop and leaves its RX stopped on
  *         successful cleanup. It does not configure clocks/MPU or HAL callbacks.
  */
HAL_StatusTypeDef ELRS_LoopbackTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_ELRS_ELRS_TEST_H_ */
