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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define THERM_0_Pin GPIO_PIN_0
#define THERM_0_GPIO_Port GPIOA
#define THERM_1_Pin GPIO_PIN_1
#define THERM_1_GPIO_Port GPIOA
#define THERM_2_Pin GPIO_PIN_2
#define THERM_2_GPIO_Port GPIOA
#define MOTOR_1_PWM_Pin GPIO_PIN_3
#define MOTOR_1_PWM_GPIO_Port GPIOA
#define QUAD0_B_Pin GPIO_PIN_4
#define QUAD0_B_GPIO_Port GPIOA
#define SERVO_0_PWM_Pin GPIO_PIN_5
#define SERVO_0_PWM_GPIO_Port GPIOA
#define QUAD_0_A_Pin GPIO_PIN_6
#define QUAD_0_A_GPIO_Port GPIOA
#define HEATER_0_Pin GPIO_PIN_7
#define HEATER_0_GPIO_Port GPIOA
#define HEATER_1_Pin GPIO_PIN_4
#define HEATER_1_GPIO_Port GPIOC
#define HEATER_2_Pin GPIO_PIN_0
#define HEATER_2_GPIO_Port GPIOB
#define MOTOR_1_DIR_Pin GPIO_PIN_1
#define MOTOR_1_DIR_GPIO_Port GPIOB
#define MOTOR_1_NDIR_Pin GPIO_PIN_2
#define MOTOR_1_NDIR_GPIO_Port GPIOB
#define SERVO_2_PWM_Pin GPIO_PIN_10
#define SERVO_2_PWM_GPIO_Port GPIOB
#define WHITE_LED_Pin GPIO_PIN_12
#define WHITE_LED_GPIO_Port GPIOB
#define UV_LED_Pin GPIO_PIN_13
#define UV_LED_GPIO_Port GPIOB
#define MOTOR_0_PWM_Pin GPIO_PIN_14
#define MOTOR_0_PWM_GPIO_Port GPIOB
#define MOTOR_0_NDIR_Pin GPIO_PIN_15
#define MOTOR_0_NDIR_GPIO_Port GPIOB
#define MOTOR_0_DIR_Pin GPIO_PIN_6
#define MOTOR_0_DIR_GPIO_Port GPIOC
#define SERVO_1_PWM_Pin GPIO_PIN_3
#define SERVO_1_PWM_GPIO_Port GPIOB
#define QUAD_1_A_Pin GPIO_PIN_6
#define QUAD_1_A_GPIO_Port GPIOB
#define QUAD1_B_Pin GPIO_PIN_7
#define QUAD1_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
