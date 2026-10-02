/*
 * elrs.c
 *
 * Author: Viết Khoa
 */

#include "elrs.h"
#include "usart.h"
#include <string.h>

#define ELRS_DMA_ALIGNMENT                       32U
#define ELRS_DMA_REGION_BASE                     0x30000000UL
#define ELRS_DMA_REGION_SIZE                     4096UL
#define ELRS_DMA_MPU_REGION                      1U
#define ELRS_DMA_MASK                            (ELRS_RX_DMA_BUFFER_SIZE - 1U)
#define ELRS_FIFO_MASK                           (ELRS_RX_FIFO_SIZE - 1U)

_Static_assert((ELRS_RX_FIFO_SIZE != 0U) &&
               ((ELRS_RX_FIFO_SIZE & ELRS_FIFO_MASK) == 0U),
               "ELRS FIFO size must be a nonzero power of two");
_Static_assert((ELRS_RX_DMA_BUFFER_SIZE >= 2U) &&
               ((ELRS_RX_DMA_BUFFER_SIZE & ELRS_DMA_MASK) == 0U),
               "ELRS DMA size must be a power of two and at least two bytes");
_Static_assert(ELRS_RX_DMA_BUFFER_SIZE <= ELRS_DMA_REGION_SIZE,
               "UART6 DMA buffer exceeds its reserved MPU region");

/* Non-cacheable USART6 RX DMA buffer in SRAM1. */
static uint8_t uart6_rx_dma_buffer[ELRS_RX_DMA_BUFFER_SIZE]
    __attribute__((section(".uart6_dma"), aligned(ELRS_DMA_ALIGNMENT), used));

/* Linker boundaries of the UART6 DMA section. */
extern uint8_t __uart6_dma_start__;
extern uint8_t __uart6_dma_end__;

/* Single ISR producer and single task consumer. */
static uint8_t rx_fifo[ELRS_RX_FIFO_SIZE];
static volatile uint32_t fifo_head;
static volatile uint32_t fifo_tail;
static uint16_t dma_old_pos;
static uint8_t initialized;
static volatile uint8_t rx_running;
static volatile uint8_t rx_fault;

static uint8_t ELRS_ConfigIsValid(void);
static uint8_t ELRS_MemoryIsValid(void);

/**
  * @brief  Validate USART6 and circular RX DMA configuration.
  */
static uint8_t ELRS_ConfigIsValid(void)
{
    const DMA_HandleTypeDef *dma = huart6.hdmarx;

    if ((huart6.Instance != USART6) ||
        (huart6.Init.WordLength != UART_WORDLENGTH_8B) ||
        (huart6.Init.Parity != UART_PARITY_NONE) ||
        (huart6.Init.StopBits != UART_STOPBITS_1) ||
        ((huart6.Init.Mode & UART_MODE_RX) == 0U))
    {
        return 0U;
    }

    if ((dma == NULL) ||
        (dma->Init.Request != DMA_REQUEST_USART6_RX) ||
        (dma->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (dma->Init.Mode != DMA_CIRCULAR) ||
        (dma->Init.FIFOMode != DMA_FIFOMODE_DISABLE) ||
        (dma->Init.PeriphInc != DMA_PINC_DISABLE) ||
        (dma->Init.MemInc != DMA_MINC_ENABLE) ||
        (dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) ||
        (dma->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE))
    {
        return 0U;
    }

    return 1U;
}

/**
  * @brief  Validate DMA placement and non-cacheable MPU attributes.
  */
static uint8_t ELRS_MemoryIsValid(void)
{
    const uintptr_t address = (uintptr_t)uart6_rx_dma_buffer;
    const uintptr_t section_start = (uintptr_t)&__uart6_dma_start__;
    const uintptr_t section_end = (uintptr_t)&__uart6_dma_end__;
    const uint32_t saved_region = MPU->RNR;
    const uint32_t required_mask = MPU_RASR_ENABLE_Msk | MPU_RASR_SIZE_Msk |
        MPU_RASR_SRD_Msk | MPU_RASR_AP_Msk | MPU_RASR_TEX_Msk |
        MPU_RASR_C_Msk | MPU_RASR_B_Msk;
    const uint32_t required_value = MPU_RASR_ENABLE_Msk |
        (11UL << MPU_RASR_SIZE_Pos) | (3UL << MPU_RASR_AP_Pos) |
        (1UL << MPU_RASR_TEX_Pos);
    const uint32_t mpu_control = MPU->CTRL;
    uint32_t mpu_base;
    uint32_t mpu_attributes;

    MPU->RNR = ELRS_DMA_MPU_REGION;
    __DSB();
    mpu_base = MPU->RBAR & MPU_RBAR_ADDR_Msk;
    mpu_attributes = MPU->RASR;
    MPU->RNR = saved_region;

    if ((RCC->AHB2ENR & RCC_AHB2ENR_D2SRAM1EN) == 0U)
    {
        return 0U;
    }

    if ((section_start < ELRS_DMA_REGION_BASE) ||
        (section_end > ELRS_DMA_REGION_BASE + ELRS_DMA_REGION_SIZE) ||
        (address < section_start) ||
        (address > section_end) ||
        (sizeof(uart6_rx_dma_buffer) > section_end - address) ||
        ((address & (ELRS_DMA_ALIGNMENT - 1U)) != 0U))
    {
        return 0U;
    }

    /* Full-access normal memory, non-cacheable, with all subregions enabled. */
    if (((mpu_control & MPU_CTRL_ENABLE_Msk) == 0U) ||
        (mpu_base != ELRS_DMA_REGION_BASE) ||
        ((mpu_attributes & required_mask) != required_value))
    {
        return 0U;
    }

    return 1U;
}

/**
  * @brief  Validate configuration and initialize RX storage.
  */
HAL_StatusTypeDef ELRS_Init(void)
{
    if ((rx_running != 0U) || (huart6.RxState != HAL_UART_STATE_READY))
    {
        return HAL_BUSY;
    }

    initialized = 0U;

    if ((ELRS_ConfigIsValid() == 0U) || (ELRS_MemoryIsValid() == 0U))
    {
        rx_fault = 1U;
        return HAL_ERROR;
    }

    memset(uart6_rx_dma_buffer, 0, sizeof(uart6_rx_dma_buffer));
    __DSB();

    fifo_head = 0U;
    fifo_tail = 0U;
    dma_old_pos = 0U;
    rx_fault = 0U;

    initialized = 1U;
    return HAL_OK;
}

/**
  * @brief  Start circular RX DMA with HT, TC and IDLE events.
  */
HAL_StatusTypeDef ELRS_Receive_DMA_Start(void)
{
    HAL_StatusTypeDef status;

    if (initialized == 0U)
    {
        return HAL_ERROR;
    }

    if ((rx_running != 0U) || (huart6.RxState != HAL_UART_STATE_READY))
    {
        return HAL_BUSY;
    }

    dma_old_pos = 0U;
    fifo_head = 0U;
    fifo_tail = 0U;
    __HAL_UART_CLEAR_FLAG(&huart6,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF |
        UART_CLEAR_PEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);
    __DSB();

    /* Ownership is published before DMA interrupts become active. */
    rx_running = 1U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart6, uart6_rx_dma_buffer, ELRS_RX_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        rx_running = 0U;
        rx_fault = 1U;
    }

    return status;
}

/**
  * @brief  Stop RX while preserving UART TX state.
  */
HAL_StatusTypeDef ELRS_Receive_DMA_Stop(void)
{
    HAL_StatusTypeDef status;

    if (rx_running == 0U)
    {
        return HAL_OK;
    }

    rx_running = 0U;
    __DMB();
    status = HAL_UART_AbortReceive(&huart6);
    if (status != HAL_OK)
    {
        /* Failed abort retains RX ownership. */
        rx_running = 1U;
        rx_fault = 1U;
    }

    return status;
}

/**
  * @brief  Return the unread FIFO byte count.
  */
uint32_t ELRS_Available(void)
{
    const uint32_t tail = fifo_tail;
    const uint32_t head = fifo_head;

    return head - tail;
}

/**
  * @brief  Copy available FIFO bytes without waiting.
  */
uint16_t ELRS_Read(uint8_t *data, uint16_t capacity)
{
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t index;
    uint32_t first;

    if ((data == NULL) || (capacity == 0U))
    {
        return 0U;
    }

    tail = fifo_tail;
    head = fifo_head;
    __DMB(); /* Acquire bytes published by the producer. */
    count = head - tail;
    if (count > capacity)
    {
        count = capacity;
    }

    if (count == 0U)
    {
        return 0U;
    }

    index = tail & ELRS_FIFO_MASK;
    first = ELRS_RX_FIFO_SIZE - index;
    if (first > count)
    {
        first = count;
    }
    memcpy(data, &rx_fifo[index], first);
    if (count > first)
    {
        memcpy(&data[first], rx_fifo, count - first);
    }

    __DMB(); /* Release FIFO slots after copying. */
    fifo_tail = tail + count;
    return (uint16_t)count;
}

/**
  * @brief  Return the latched transport fault.
  */
uint8_t ELRS_HasRxFault(void)
{
    return rx_fault;
}

/**
  * @brief  Copy newly received DMA bytes into the software FIFO.
  */
void ELRS_UART_RxEventHandler(UART_HandleTypeDef *huart, uint16_t size)
{
    uint32_t remaining;
    uint32_t pos;
    uint32_t count;
    uint32_t head;
    uint32_t tail;
    uint32_t free_bytes;
    uint32_t copy_count;
    const volatile uint8_t *dma = uart6_rx_dma_buffer;

    if ((huart != &huart6) || (rx_running == 0U))
    {
        return;
    }

    /* Live NDTR provides the current position for HT, TC and IDLE events. */
    (void)size;
    remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > ELRS_RX_DMA_BUFFER_SIZE)
    {
        rx_fault = 1U;
        return;
    }

    pos = (ELRS_RX_DMA_BUFFER_SIZE - remaining) & ELRS_DMA_MASK;
    __DMB();
    count = (pos - (uint32_t)dma_old_pos) & ELRS_DMA_MASK;

    if (count == 0U)
    {
        return;
    }

    head = fifo_head;
    tail = fifo_tail;
    __DMB(); /* Acquire slots released by the consumer. */
    free_bytes = ELRS_RX_FIFO_SIZE - (head - tail);
    copy_count = count;
    if (copy_count > free_bytes)
    {
        copy_count = free_bytes;
    }

    for (uint32_t i = 0U; i < copy_count; ++i)
    {
        rx_fifo[(head + i) & ELRS_FIFO_MASK] =
            dma[((uint32_t)dma_old_pos + i) & ELRS_DMA_MASK];
    }

    __DMB(); /* Publish completed FIFO writes. */
    fifo_head = head + copy_count;

    /* Excess new bytes are dropped; unread bytes remain intact. */
    if (copy_count != count)
    {
        rx_fault = 1U;
    }

    dma_old_pos = (uint16_t)pos;
}

/**
  * @brief  Latch UART and DMA receive errors.
  */
void ELRS_UART_ErrorHandler(UART_HandleTypeDef *huart)
{
    if ((huart != &huart6) || (rx_running == 0U))
    {
        return;
    }

    rx_fault = 1U;
}
