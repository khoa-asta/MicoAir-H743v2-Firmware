/*
 * dshot300.c
 *
 * Author: Viết Khoa
 */

#include "dshot300.h"
#include "tim.h"
#include <stddef.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_tim1_up;
extern uint8_t __dshot300_dma_region_start__[];
extern uint8_t __dshot300_dma_region_end__[];

#define DSHOT_REGION_BASE       0x30001000UL
#define DSHOT_REGION_END        0x30001800UL
#define DSHOT_PERIOD            799U
#define DSHOT_GPIO_PINS         (GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14)
#define DSHOT_GPIO_MODE_MASK    ((3UL << 18) | (3UL << 22) | (3UL << 26) | (3UL << 28))
#define DSHOT_GPIO_AF_MODE      ((2UL << 18) | (2UL << 22) | (2UL << 26) | (2UL << 28))
#define DSHOT_GPIO_PULLUP       ((1UL << 18) | (1UL << 22) | (1UL << 26) | (1UL << 28))
#define DSHOT_GPIO_AFR_MASK     ((15UL << 4) | (15UL << 12) | (15UL << 20) | (15UL << 24))
#define DSHOT_GPIO_AFR_VALUE    ((1UL << 4) | (1UL << 12) | (1UL << 20) | (1UL << 24))
#define DSHOT_CC_ENABLE        (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)
#define DSHOT_CC_POLARITY      (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P)
#define DSHOT_DMA_COUNT        ((DSHOT300_TX_ROWS - 1U) * DSHOT300_MOTOR_COUNT)

/* PWM rows: CCR1 to CCR4 correspond to M4 to M1. */
static uint16_t s_tx[DSHOT300_TX_ROWS][DSHOT300_MOTOR_COUNT]
    __attribute__((section(".dshot300_dma"), aligned(32), used));

/* Transport state and completed-transfer sequence. */
static volatile DSHOT300_State_t s_state;
static volatile uint32_t s_completed_count;
static volatile uint32_t s_tx_tick;
static volatile uint32_t s_release_tick;
static uint8_t s_owned;

_Static_assert(sizeof(s_tx) == 144U, "Unexpected DShot buffer size");
_Static_assert(sizeof(s_tx) <= 2048U, "DShot DMA region too small");

static void DSHOT300_DmaComplete(DMA_HandleTypeDef *hdma);
static void DSHOT300_DmaError(DMA_HandleTypeDef *hdma);

/**
  * @brief  Encode a throttle value with the bidirectional checksum.
  */
static uint16_t DSHOT300_Encode(uint16_t value)
{
    /* The separate telemetry-request bit is zero. */
    const uint16_t payload = (uint16_t)(value << 1U);
    const uint16_t checksum = (uint16_t)(~(payload ^ (payload >> 4U)
                                          ^ (payload >> 8U)) & 0x0FU);
    return (uint16_t)((payload << 4U) | checksum);
}

/**
  * @brief  Build PWM rows for logical motors M1 to M4.
  */
static void DSHOT300_Build(const uint16_t values[DSHOT300_MOTOR_COUNT])
{
    for (uint32_t motor = 0U; motor < DSHOT300_MOTOR_COUNT; ++motor)
    {
        const uint16_t frame = DSHOT300_Encode(values[motor]);
        const uint32_t channel = 3U - motor;

        for (uint32_t bit = 0U; bit < DSHOT300_FRAME_BITS; ++bit)
        {
            if ((frame & (0x8000U >> bit)) != 0U)
            {
                s_tx[bit][channel] = DSHOT300_BIT1_TICKS;
            }
            else
            {
                s_tx[bit][channel] = DSHOT300_BIT0_TICKS;
            }
        }

        s_tx[DSHOT300_FRAME_BITS][channel] = 0U;
        s_tx[DSHOT300_FRAME_BITS + 1U][channel] = 0U;
    }
}

/**
  * @brief  Release motor outputs with pull-ups enabled.
  */
static void DSHOT300_ReleaseLines(void)
{
    GPIOE->PUPDR = (GPIOE->PUPDR & ~(uint32_t)DSHOT_GPIO_MODE_MASK) | DSHOT_GPIO_PULLUP;
    GPIOE->MODER &= ~(uint32_t)DSHOT_GPIO_MODE_MASK;
    __DSB();
}

/**
  * @brief  Stop TIM1 requests and release motor outputs.
  */
static void DSHOT300_StopTimer(void)
{
    TIM1->DIER = 0U;
    TIM1->CR1 &= ~(uint32_t)TIM_CR1_CEN;
    DSHOT300_ReleaseLines();
    TIM1->CCER &= ~(uint32_t)DSHOT_CC_ENABLE;
    TIM1->BDTR &= ~(uint32_t)TIM_BDTR_MOE;
}

/**
  * @brief  Validate the DMA buffer address and alignment.
  */
static uint8_t DSHOT300_MemoryValid(void)
{
    const uintptr_t first = (uintptr_t)s_tx;
    const uintptr_t last = first + sizeof(s_tx);

    return ((uintptr_t)__dshot300_dma_region_start__ == DSHOT_REGION_BASE)
        && ((uintptr_t)__dshot300_dma_region_end__ == DSHOT_REGION_END)
        && (first >= DSHOT_REGION_BASE) && (last <= DSHOT_REGION_END)
        && ((first & 31U) == 0U);
}

/**
  * @brief  Validate non-cacheable MPU coverage of the DMA buffer.
  */
static uint8_t DSHOT300_MpuValid(void)
{
    const uint32_t primask = __get_PRIMASK();
    uint32_t valid = 1U;
    __disable_irq();
    const uint32_t saved_rnr = MPU->RNR;
    const uint32_t region_count = (MPU->TYPE >> 8U) & 0xFFU;
    if (((MPU->CTRL & 1U) == 0U) || (region_count < 3U) || (region_count > 16U))
    {
        valid = 0U;
    }
    else
    {
        MPU->RNR = 2U;
        const uint32_t rbar = MPU->RBAR;
        const uint32_t rasr = MPU->RASR;

        /* Full-access normal memory, non-cacheable, with all subregions enabled. */
        const uint32_t checked = (7UL << 24) | (7UL << 19) | (3UL << 16)
                               | (0xFFUL << 8) | (31UL << 1) | 1UL;
        const uint32_t expected = (3UL << 24) | (1UL << 19) | (10UL << 1) | 1UL;
        valid = ((rbar & 0xFFFFFFE0UL) == DSHOT_REGION_BASE)
             && ((rasr & checked) == expected);

        /* Higher-priority regions must not overlap the DMA region. */
        for (uint32_t r = 3U; valid && (r < region_count); ++r)
        {
            MPU->RNR = r;
            const uint32_t attr = MPU->RASR;
            if ((attr & 1U) != 0U)
            {
                const uint32_t size = (attr >> 1U) & 31U;
                const uint64_t bytes = 1ULL << (size + 1U);
                const uint64_t base = ((uint64_t)(MPU->RBAR & 0xFFFFFFE0UL))
                                    & ~(bytes - 1ULL);
                if ((base < DSHOT_REGION_END) && ((base + bytes) > DSHOT_REGION_BASE))
                {
                    valid = 0U;
                }
            }
        }
    }

    MPU->RNR = saved_rnr;
    __set_PRIMASK(primask);
    return (uint8_t)valid;
}

/**
  * @brief  Validate clock, timer, DMA, GPIO and memory configuration.
  */
static uint8_t DSHOT300_ConfigIsValid(void)
{
    if ((HAL_RCC_GetHCLKFreq() != 240000000UL)
        || (HAL_RCC_GetPCLK2Freq() != 120000000UL)
        || ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) != RCC_APB2_DIV2))
    {
        return 0U;
    }

    const uint32_t mode_mask1 = TIM_CCMR1_CC1S | TIM_CCMR1_CC2S
                             | TIM_CCMR1_OC1M | TIM_CCMR1_OC2M
                             | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    const uint32_t mode_mask2 = TIM_CCMR2_CC3S | TIM_CCMR2_CC4S
                             | TIM_CCMR2_OC3M | TIM_CCMR2_OC4M
                             | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    const uint32_t pwm_modes = TIM_OCMODE_PWM1 | (TIM_OCMODE_PWM1 << 8U);
    if ((htim1.Instance != TIM1) || ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0U)
        || (TIM1->PSC != 0U) || (TIM1->ARR != DSHOT_PERIOD) || (TIM1->RCR != 0U)
        || ((TIM1->CR1 & (TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD
                         | TIM_CR1_UDIS | TIM_CR1_OPM)) != TIM_CR1_ARPE)
        || ((TIM1->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U)
        || ((TIM1->CCMR1 & mode_mask1) != (pwm_modes | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE))
        || ((TIM1->CCMR2 & mode_mask2) != (pwm_modes | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE))
        || ((TIM1->CCER & DSHOT_CC_POLARITY) != DSHOT_CC_POLARITY)
        || ((TIM1->BDTR & (TIM_BDTR_BKE | TIM_BDTR_BK2E | TIM_BDTR_LOCK)) != 0U))
    {
        return 0U;
    }

    const uint32_t dma_mask = DMA_SxCR_DIR | DMA_SxCR_PINC | DMA_SxCR_MINC
                           | DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_CIRC
                           | DMA_SxCR_DBM | DMA_SxCR_PL;
    const uint32_t dma_expected = DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE
                               | DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD
                               | DMA_PRIORITY_VERY_HIGH;
    if ((hdma_tim1_up.Instance != DMA2_Stream3)
        || (hdma_tim1_up.Init.Request != DMA_REQUEST_TIM1_UP)
        || (htim1.hdma[TIM_DMA_ID_UPDATE] != &hdma_tim1_up)
        || ((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) == 0U)
        || ((DMA2_Stream3->CR & dma_mask) != dma_expected)
        || ((DMA2_Stream3->FCR & DMA_SxFCR_DMDIS) != 0U))
    {
        return 0U;
    }

    if (((RCC->AHB4ENR & RCC_AHB4ENR_GPIOEEN) == 0U)
        || ((GPIOE->AFR[1] & DSHOT_GPIO_AFR_MASK) != DSHOT_GPIO_AFR_VALUE)
        || (!s_owned && ((GPIOE->MODER & DSHOT_GPIO_MODE_MASK) != DSHOT_GPIO_AF_MODE)))
    {
        return 0U;
    }

    if (!DSHOT300_MemoryValid())
    {
        return 0U;
    }

    if (!DSHOT300_MpuValid())
    {
        return 0U;
    }

    if (NVIC_GetEnableIRQ(DMA2_Stream3_IRQn) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/**
  * @brief  Validate hardware and initialize DShot transport.
  */
HAL_StatusTypeDef DSHOT300_Init(void)
{
    if ((s_state == DSHOT300_TX) || (s_state == DSHOT300_QUIET)
        || ((TIM1->CR1 & TIM_CR1_CEN) != 0U)
        || ((DMA2_Stream3->CR & DMA_SxCR_EN) != 0U))
    {
        return HAL_BUSY;
    }
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    if (DSHOT300_ConfigIsValid() == 0U)
    {
        s_state = DSHOT300_FAULT;
        return HAL_ERROR;
    }

    if (s_owned && (HAL_DMA_GetState(&hdma_tim1_up) == HAL_DMA_STATE_BUSY))
    {
        if (HAL_DMA_Abort(&hdma_tim1_up) != HAL_OK)
        {
            s_state = DSHOT300_FAULT;
            return HAL_ERROR;
        }
    }

    if (HAL_DMA_GetState(&hdma_tim1_up) != HAL_DMA_STATE_READY)
    {
        s_state = DSHOT300_FAULT;
        return HAL_ERROR;
    }

    s_owned = 1U;
    DSHOT300_StopTimer();
    memset(s_tx, 0, sizeof(s_tx));
    __DSB();
    hdma_tim1_up.XferCpltCallback = DSHOT300_DmaComplete;
    hdma_tim1_up.XferHalfCpltCallback = NULL;
    hdma_tim1_up.XferErrorCallback = DSHOT300_DmaError;
    hdma_tim1_up.XferAbortCallback = NULL;
    s_state = DSHOT300_READY;
    return HAL_OK;
}

/**
  * @brief  Transmit stop or throttle values for four motors.
  */
HAL_StatusTypeDef DSHOT300_Send(const uint16_t motor_values[DSHOT300_MOTOR_COUNT])
{
    if (motor_values == NULL)
    {
        return HAL_ERROR;
    }

    for (uint32_t m = 0U; m < DSHOT300_MOTOR_COUNT; ++m)
    {
        if ((motor_values[m] > 2047U) || ((motor_values[m] != 0U) && (motor_values[m] < 48U)))
        {
            return HAL_ERROR;
        }
    }
    DSHOT300_Process();
    if (DSHOT300_IsBusy())
    {
        return HAL_BUSY;
    }

    if (s_state != DSHOT300_READY)
    {
        return HAL_ERROR;
    }
    DSHOT300_Build(motor_values);

    DSHOT300_StopTimer();

    /* Active idle level before loading the first bit into preload. */
    TIM1->PSC = 0U;
    TIM1->ARR = DSHOT_PERIOD;
    TIM1->RCR = 0U;
    TIM1->CNT = 0U;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;

    /* The first natural update activates the first bit. */
    TIM1->CCR1 = s_tx[0][0];
    TIM1->CCR2 = s_tx[0][1];
    TIM1->CCR3 = s_tx[0][2];
    TIM1->CCR4 = s_tx[0][3];
    TIM1->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    __DSB();
    const HAL_StatusTypeDef result = HAL_DMA_Start_IT(
        &hdma_tim1_up,
        (uint32_t)(uintptr_t)&s_tx[1][0],
        (uint32_t)(uintptr_t)&TIM1->DMAR,
        DSHOT_DMA_COUNT);
    if (result != HAL_OK)
    {
        DSHOT300_StopTimer();
        s_state = DSHOT300_FAULT;
        return result;
    }

    s_tx_tick = HAL_GetTick();
    s_state = DSHOT300_TX;
    GPIOE->OTYPER &= ~(uint32_t)DSHOT_GPIO_PINS;
    GPIOE->OSPEEDR |= DSHOT_GPIO_MODE_MASK;
    TIM1->CCER |= DSHOT_CC_ENABLE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    GPIOE->MODER = (GPIOE->MODER & ~(uint32_t)DSHOT_GPIO_MODE_MASK) | DSHOT_GPIO_AF_MODE;
    TIM1->DIER = TIM_DIER_UDE;
    __DSB();
    TIM1->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

/**
  * @brief  Release outputs and enter the quiet interval.
  */
static void DSHOT300_DmaComplete(DMA_HandleTypeDef *hdma)
{
    if ((hdma != &hdma_tim1_up) || (s_state != DSHOT300_TX))
    {
        return;
    }

    DSHOT300_StopTimer();
    ++s_completed_count;
    s_release_tick = HAL_GetTick();
    __DMB();
    s_state = DSHOT300_QUIET;
}

/**
  * @brief  Stop output and latch a DMA transfer fault.
  */
static void DSHOT300_DmaError(DMA_HandleTypeDef *hdma)
{
    if ((hdma != &hdma_tim1_up) || (s_state != DSHOT300_TX))
    {
        return;
    }
    DSHOT300_StopTimer();

    ((DMA_Stream_TypeDef *)hdma->Instance)->CR &= ~(uint32_t)DMA_SxCR_EN;
    __DMB();
    s_state = DSHOT300_FAULT;
}

/**
  * @brief  Advance the quiet interval and handle transmit timeout.
  */
void DSHOT300_Process(void)
{
    if ((s_state == DSHOT300_QUIET)
        && ((uint32_t)(HAL_GetTick() - s_release_tick) >= DSHOT300_QUIET_TICKS))
    {
        s_state = DSHOT300_READY;
    }

    if ((s_state == DSHOT300_TX)
        && ((uint32_t)(HAL_GetTick() - s_tx_tick) >= DSHOT300_TX_TIMEOUT_MS))
    {
        /* IRQ exclusion prevents completion from racing timeout handling. */
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint8_t expired = (s_state == DSHOT300_TX);
        if (expired)
        {
            DSHOT300_StopTimer();
            s_state = DSHOT300_FAULT;
        }
        __set_PRIMASK(primask);
        if (expired)
        {
            /* Blocking DMA abort runs after IRQ state is restored. */
            (void)HAL_DMA_Abort(&hdma_tim1_up);
        }
    }
}

/**
  * @brief  Report an active transfer or quiet interval.
  */
uint8_t DSHOT300_IsBusy(void)
{
    return (s_state == DSHOT300_TX) || (s_state == DSHOT300_QUIET);
}

/**
  * @brief  Cancel transport without transmitting a stop frame.
  */
HAL_StatusTypeDef DSHOT300_Abort(void)
{
    if (!s_owned)
    {
        return HAL_ERROR;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t had_fault = (s_state == DSHOT300_FAULT);
    DSHOT300_StopTimer();
    s_state = DSHOT300_FAULT;
    __set_PRIMASK(primask);
    HAL_StatusTypeDef result = HAL_OK;
    if ((HAL_DMA_GetState(&hdma_tim1_up) == HAL_DMA_STATE_BUSY)
        || ((DMA2_Stream3->CR & DMA_SxCR_EN) != 0U))
    {
        result = HAL_DMA_Abort(&hdma_tim1_up);
    }
    if (result != HAL_OK)
    {
        return result;
    }

    s_release_tick = HAL_GetTick();

    /* Existing faults remain latched until successful initialization. */
    if (had_fault == 0U)
    {
        s_state = DSHOT300_QUIET;
    }

    return HAL_OK;
}

/**
  * @brief  Return the current transport state.
  */
DSHOT300_State_t DSHOT300_GetState(void)
{
    return s_state;
}

/**
  * @brief  Return the sequence count of completed DMA transfers.
  */
uint32_t DSHOT300_GetCompletedCount(void)
{
    return s_completed_count;
}
