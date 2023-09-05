/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIMER_ONE_MS 1000
#define I2C_ADDRESS 0x01
#define SCIENCE_TEMP_0_Pin GPIO_PIN_0
#define SCIENCE_TEMP_0_GPIO_Port GPIOA
#define SCIENCE_TEMP_1_Pin GPIO_PIN_1
#define SCIENCE_TEMP_1_GPIO_Port GPIOA
#define SCIENCE_TEMP_2_Pin GPIO_PIN_2
#define SCIENCE_TEMP_2_GPIO_Port GPIOA
#define DIAG_CURR_0_Pin GPIO_PIN_3
#define DIAG_CURR_0_GPIO_Port GPIOA
#define DIAG_CURR_1_Pin GPIO_PIN_4
#define DIAG_CURR_1_GPIO_Port GPIOA
#define DIAG_CURR_2_Pin GPIO_PIN_5
#define DIAG_CURR_2_GPIO_Port GPIOA
#define DIAG_TEMP_0_Pin GPIO_PIN_6
#define DIAG_TEMP_0_GPIO_Port GPIOA
#define DIAG_TEMP_1_Pin GPIO_PIN_7
#define DIAG_TEMP_1_GPIO_Port GPIOA
#define DIAG_TEMP_2_Pin GPIO_PIN_4
#define DIAG_TEMP_2_GPIO_Port GPIOC
#define SERVO_0_Pin GPIO_PIN_9
#define SERVO_0_GPIO_Port GPIOE
#define SERVO_1_Pin GPIO_PIN_11
#define SERVO_1_GPIO_Port GPIOE
#define SERVO_2_Pin GPIO_PIN_13
#define SERVO_2_GPIO_Port GPIOE
#define MOSFET_0_Pin GPIO_PIN_12
#define MOSFET_0_GPIO_Port GPIOB
#define MOSFET_1_Pin GPIO_PIN_13
#define MOSFET_1_GPIO_Port GPIOB
#define MOSFET_2_Pin GPIO_PIN_14
#define MOSFET_2_GPIO_Port GPIOB
#define MOSFET_3_Pin GPIO_PIN_15
#define MOSFET_3_GPIO_Port GPIOB
#define MOSFET_4_Pin GPIO_PIN_8
#define MOSFET_4_GPIO_Port GPIOD
#define MOSFET_5_Pin GPIO_PIN_9
#define MOSFET_5_GPIO_Port GPIOD
#define MOSFET_6_Pin GPIO_PIN_10
#define MOSFET_6_GPIO_Port GPIOD
#define MOSFET_7_Pin GPIO_PIN_11
#define MOSFET_7_GPIO_Port GPIOD
#define MOSFET_8_Pin GPIO_PIN_12
#define MOSFET_8_GPIO_Port GPIOD
#define MOSFET_9_Pin GPIO_PIN_13
#define MOSFET_9_GPIO_Port GPIOD
#define MOSFET_11_Pin GPIO_PIN_14
#define MOSFET_11_GPIO_Port GPIOD
#define MOSFET_10_Pin GPIO_PIN_15
#define MOSFET_10_GPIO_Port GPIOD
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define DEBUG_LED_3_Pin GPIO_PIN_6
#define DEBUG_LED_3_GPIO_Port GPIOD
#define DEBUG_LED_2_Pin GPIO_PIN_7
#define DEBUG_LED_2_GPIO_Port GPIOD
#define DEBUG_LED_1_Pin GPIO_PIN_3
#define DEBUG_LED_1_GPIO_Port GPIOB
#define DEBUG_LED_0_Pin GPIO_PIN_4
#define DEBUG_LED_0_GPIO_Port GPIOB
#define I2C1_SPECTRAL_SCL_Pin GPIO_PIN_6
#define I2C1_SPECTRAL_SCL_GPIO_Port GPIOB
#define I2C1_SPECTRAL_SDA_Pin GPIO_PIN_7
#define I2C1_SPECTRAL_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
