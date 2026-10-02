/*
 * elrs.h
 *
 * Author: Viết Khoa
 */

#ifndef DEVICE_ELRS_ELRS_H_
#define DEVICE_ELRS_ELRS_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_RX_DMA_BUFFER_SIZE    512U
#define ELRS_RX_FIFO_SIZE          2048U

/* Validate configuration and initialize RX storage. */
HAL_StatusTypeDef ELRS_Init(void);

/* Start circular RX DMA with HT, TC and IDLE events. */
HAL_StatusTypeDef ELRS_Receive_DMA_Start(void);

/* Stop RX while preserving UART TX state. */
HAL_StatusTypeDef ELRS_Receive_DMA_Stop(void);

/* Copy available FIFO bytes without waiting. */
uint16_t ELRS_Read(uint8_t *data, uint16_t capacity);

/* Return the unread FIFO byte count. */
uint32_t ELRS_Available(void);

/* Return the transport fault latched until successful initialization. */
uint8_t ELRS_HasRxFault(void);

/* Copy new DMA bytes into the FIFO from the receive callback. */
void ELRS_UART_RxEventHandler(UART_HandleTypeDef *huart, uint16_t size);

/* Latch UART and DMA receive errors from the error callback. */
void ELRS_UART_ErrorHandler(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_ELRS_ELRS_H_ */
