/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_3
#define LED_RED_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_4
#define LED_BLUE_GPIO_Port GPIOE
#define BMI088_ACCEL_DRDY_Pin GPIO_PIN_14
#define BMI088_ACCEL_DRDY_GPIO_Port GPIOC
#define BMI088_ACCEL_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define BMI088_GYRO_DRDY_Pin GPIO_PIN_15
#define BMI088_GYRO_DRDY_GPIO_Port GPIOC
#define BMI088_GYRO_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define VBAT_ADC1_IN10_Pin GPIO_PIN_0
#define VBAT_ADC1_IN10_GPIO_Port GPIOC
#define CURRENT_ADC1_IN11_Pin GPIO_PIN_1
#define CURRENT_ADC1_IN11_GPIO_Port GPIOC
#define MTF01_UART4_TX_Pin GPIO_PIN_0
#define MTF01_UART4_TX_GPIO_Port GPIOA
#define MTF01_UART4_RX_Pin GPIO_PIN_1
#define MTF01_UART4_RX_GPIO_Port GPIOA
#define MOTOR4_DSHOT_Pin GPIO_PIN_9
#define MOTOR4_DSHOT_GPIO_Port GPIOE
#define MOTOR3_DSHOT_Pin GPIO_PIN_11
#define MOTOR3_DSHOT_GPIO_Port GPIOE
#define MOTOR2_DSHOT_Pin GPIO_PIN_13
#define MOTOR2_DSHOT_GPIO_Port GPIOE
#define MOTOR1_DSHOT_Pin GPIO_PIN_14
#define MOTOR1_DSHOT_GPIO_Port GPIOE
#define GPS_UART3_TX_Pin GPIO_PIN_8
#define GPS_UART3_TX_GPIO_Port GPIOD
#define GPS_UART3_RX_Pin GPIO_PIN_9
#define GPS_UART3_RX_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOD
#define RCIN_UART6_TX_Pin GPIO_PIN_6
#define RCIN_UART6_TX_GPIO_Port GPIOC
#define RCIN_UART6_RX_Pin GPIO_PIN_7
#define RCIN_UART6_RX_GPIO_Port GPIOC
#define TELEM_UART1_TX_Pin GPIO_PIN_9
#define TELEM_UART1_TX_GPIO_Port GPIOA
#define TELEM_UART1_RX_Pin GPIO_PIN_10
#define TELEM_UART1_RX_GPIO_Port GPIOA
#define BMI270_CS_Pin GPIO_PIN_15
#define BMI270_CS_GPIO_Port GPIOA
#define SPL06_DRDY_Pin GPIO_PIN_0
#define SPL06_DRDY_GPIO_Port GPIOD
#define SPL06_DRDY_EXTI_IRQn EXTI0_IRQn
#define BMI088_ACCEL_CS_Pin GPIO_PIN_4
#define BMI088_ACCEL_CS_GPIO_Port GPIOD
#define BMI088_GYRO_CS_Pin GPIO_PIN_5
#define BMI088_GYRO_CS_GPIO_Port GPIOD
#define BMI270_MOSI_Pin GPIO_PIN_6
#define BMI270_MOSI_GPIO_Port GPIOD
#define BMI270_SCK_Pin GPIO_PIN_3
#define BMI270_SCK_GPIO_Port GPIOB
#define BMI270_MISO_Pin GPIO_PIN_4
#define BMI270_MISO_GPIO_Port GPIOB
#define BMI270_DRDY_Pin GPIO_PIN_7
#define BMI270_DRDY_GPIO_Port GPIOB
#define BMI270_DRDY_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
