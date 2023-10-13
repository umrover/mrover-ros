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
#include "diag_curr_sensor.h"
#include "diag_temp_sensor.h"
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
#define TEMP_0_Pin GPIO_PIN_0
#define TEMP_0_GPIO_Port GPIOF
#define CURR_0_Pin GPIO_PIN_0
#define CURR_0_GPIO_Port GPIOA
#define CURR_1_Pin GPIO_PIN_1
#define CURR_1_GPIO_Port GPIOA
#define CURR_2_Pin GPIO_PIN_2
#define CURR_2_GPIO_Port GPIOA
#define CURR_3_Pin GPIO_PIN_3
#define CURR_3_GPIO_Port GPIOA
#define RA_LASER_Pin GPIO_PIN_5
#define RA_LASER_GPIO_Port GPIOA
#define UV_BULB_Pin GPIO_PIN_6
#define UV_BULB_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_7
#define RED_LED_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_4
#define GREEN_LED_GPIO_Port GPIOC
#define TEMP_4_Pin GPIO_PIN_0
#define TEMP_4_GPIO_Port GPIOB
#define TEMP_2_Pin GPIO_PIN_1
#define TEMP_2_GPIO_Port GPIOB
#define BLUE_LED_Pin GPIO_PIN_2
#define BLUE_LED_GPIO_Port GPIOB
#define TEMP_3_Pin GPIO_PIN_11
#define TEMP_3_GPIO_Port GPIOB
#define TEMP_1_Pin GPIO_PIN_12
#define TEMP_1_GPIO_Port GPIOB
#define CURR_4_Pin GPIO_PIN_14
#define CURR_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
