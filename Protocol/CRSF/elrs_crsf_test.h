/*
 * elrs_crsf_test.h
 *
 * CRSF parser checkpoint over the USART6 ELRS loopback transport.
 * Author: Viết Khoa
 */

#ifndef CRSF_ELRS_CRSF_TEST_H_
#define CRSF_ELRS_CRSF_TEST_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    CRSF_TEST_FAILURE_NONE = 0,
    CRSF_TEST_FAILURE_INIT = 1,
    CRSF_TEST_FAILURE_START = 2,
    CRSF_TEST_FAILURE_TX = 3,
    CRSF_TEST_FAILURE_TIMEOUT = 4,
    CRSF_TEST_FAILURE_DATA = 5,
    CRSF_TEST_FAILURE_CHANNELS = 6,
    CRSF_TEST_FAILURE_COUNTERS = 7,
    CRSF_TEST_FAILURE_RX = 8,
    CRSF_TEST_FAILURE_EXTRA = 9,
    CRSF_TEST_FAILURE_STOP = 10
} CRSF_TestFailure_t;

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Runs eight CRSF phases through USART6 TX-to-RX loopback.
  * @retval HAL_OK on pass; HAL_ERROR, HAL_BUSY or HAL_TIMEOUT on failure.
  * @note   Bench only: PC6/TX connected to PC7/RX, receiver disconnected.
  *         Run with IRQs and HAL tick active, after MPU/SRAM1/UART/DMA setup.
  *         Requires exclusive ownership of USART6 RX and the CRSF parser.
  *         Initializes ELRS and CRSF, runs RX, then stops its reception.
  *         A busy ELRS receiver is left untouched, including its CRSF state.
  */
HAL_StatusTypeDef ELRS_CRSFTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_ELRS_CRSF_TEST_H_ */
