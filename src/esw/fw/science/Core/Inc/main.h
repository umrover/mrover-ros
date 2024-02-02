/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void init();

void poll_spectral_status();

void update_and_send_spectral();

void update_and_send_thermistor_and_auto_shutoff_if_applicable();

void update_and_send_heater();

void receive_message();

// Used for testing while CAN isn't working
void receive_message_debug(int device, int enable);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_LED_0_Pin GPIO_PIN_13
#define DEBUG_LED_0_GPIO_Port GPIOC
#define DEBUG_LED_1_Pin GPIO_PIN_14
#define DEBUG_LED_1_GPIO_Port GPIOC
#define DEBUG_LED_2_Pin GPIO_PIN_15
#define DEBUG_LED_2_GPIO_Port GPIOC
#define THERM_N0_Pin GPIO_PIN_0
#define THERM_N0_GPIO_Port GPIOF
#define UV_LED_0_Pin GPIO_PIN_0
#define UV_LED_0_GPIO_Port GPIOA
#define UV_LED_1_Pin GPIO_PIN_1
#define UV_LED_1_GPIO_Port GPIOA
#define UV_LED_2_Pin GPIO_PIN_2
#define UV_LED_2_GPIO_Port GPIOA
#define THERM_B0_Pin GPIO_PIN_3
#define THERM_B0_GPIO_Port GPIOA
#define WHITE_LED_0_Pin GPIO_PIN_6
#define WHITE_LED_0_GPIO_Port GPIOA
#define WHITE_LED_1_Pin GPIO_PIN_7
#define WHITE_LED_1_GPIO_Port GPIOA
#define WHITE_LED_2_Pin GPIO_PIN_4
#define WHITE_LED_2_GPIO_Port GPIOC
#define THERM_N2_Pin GPIO_PIN_0
#define THERM_N2_GPIO_Port GPIOB
#define THERM_N1_Pin GPIO_PIN_1
#define THERM_N1_GPIO_Port GPIOB
#define THERM_B0B11_Pin GPIO_PIN_11
#define THERM_B0B11_GPIO_Port GPIOB
#define THERM_B1_Pin GPIO_PIN_12
#define THERM_B1_GPIO_Port GPIOB
#define HEATER_B0_Pin GPIO_PIN_13
#define HEATER_B0_GPIO_Port GPIOB
#define HEATER_N0_Pin GPIO_PIN_14
#define HEATER_N0_GPIO_Port GPIOB
#define HEATER_B1_Pin GPIO_PIN_15
#define HEATER_B1_GPIO_Port GPIOB
#define HEATER_N1_Pin GPIO_PIN_6
#define HEATER_N1_GPIO_Port GPIOC
#define HEATER_B2_Pin GPIO_PIN_8
#define HEATER_B2_GPIO_Port GPIOA
#define HEATER_N2_Pin GPIO_PIN_9
#define HEATER_N2_GPIO_Port GPIOA
#define CAN_STANDBY_Pin GPIO_PIN_15
#define CAN_STANDBY_GPIO_Port GPIOA
#define I2C_MUX_RST_Pin GPIO_PIN_7
#define I2C_MUX_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
