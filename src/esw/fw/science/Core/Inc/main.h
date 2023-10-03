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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UV_LED_0_Pin GPIO_PIN_13
#define UV_LED_0_GPIO_Port GPIOC
#define UV_LED_1_Pin GPIO_PIN_14
#define UV_LED_1_GPIO_Port GPIOC
#define UV_LED_2_Pin GPIO_PIN_15
#define UV_LED_2_GPIO_Port GPIOC
#define WHITE_LED_0_Pin GPIO_PIN_0
#define WHITE_LED_0_GPIO_Port GPIOF
#define WHITE_LED_1_Pin GPIO_PIN_1
#define WHITE_LED_1_GPIO_Port GPIOF
#define WHITE_LED_2_Pin GPIO_PIN_10
#define WHITE_LED_2_GPIO_Port GPIOG
#define THERM_0_Pin GPIO_PIN_0
#define THERM_0_GPIO_Port GPIOA
#define THERM_1_Pin GPIO_PIN_1
#define THERM_1_GPIO_Port GPIOA
#define THERM_2_Pin GPIO_PIN_2
#define THERM_2_GPIO_Port GPIOA
#define THERM_3_Pin GPIO_PIN_6
#define THERM_3_GPIO_Port GPIOA
#define THERM_4_Pin GPIO_PIN_7
#define THERM_4_GPIO_Port GPIOA
#define THERM_5_Pin GPIO_PIN_4
#define THERM_5_GPIO_Port GPIOC
#define HEATER_5_Pin GPIO_PIN_14
#define HEATER_5_GPIO_Port GPIOB
#define HEATER_4_Pin GPIO_PIN_15
#define HEATER_4_GPIO_Port GPIOB
#define HEATER_3_Pin GPIO_PIN_6
#define HEATER_3_GPIO_Port GPIOC
#define I2C_MUX_1_Pin GPIO_PIN_15
#define I2C_MUX_1_GPIO_Port GPIOA
#define I2C_MUX_0_Pin GPIO_PIN_10
#define I2C_MUX_0_GPIO_Port GPIOC
#define HEATER_5C11_Pin GPIO_PIN_11
#define HEATER_5C11_GPIO_Port GPIOC
#define HEATER_4B3_Pin GPIO_PIN_3
#define HEATER_4B3_GPIO_Port GPIOB
#define HEATER_3B4_Pin GPIO_PIN_4
#define HEATER_3B4_GPIO_Port GPIOB
#define HEATER_2_Pin GPIO_PIN_5
#define HEATER_2_GPIO_Port GPIOB
#define HEATER_1_Pin GPIO_PIN_6
#define HEATER_1_GPIO_Port GPIOB
#define HEATER_0_Pin GPIO_PIN_7
#define HEATER_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
