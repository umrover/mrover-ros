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
void HAL_PostInit();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIMIT_0_0_Pin GPIO_PIN_13
#define LIMIT_0_0_GPIO_Port GPIOC
#define LIMIT_0_1_Pin GPIO_PIN_14
#define LIMIT_0_1_GPIO_Port GPIOC
#define LIMIT_0_2_Pin GPIO_PIN_15
#define LIMIT_0_2_GPIO_Port GPIOC
#define LIMIT_0_3_Pin GPIO_PIN_0
#define LIMIT_0_3_GPIO_Port GPIOF
#define DEBUG_LED_0_Pin GPIO_PIN_1
#define DEBUG_LED_0_GPIO_Port GPIOA
#define DEBUG_LED_1_Pin GPIO_PIN_2
#define DEBUG_LED_1_GPIO_Port GPIOA
#define DEBUG_LED_2_Pin GPIO_PIN_3
#define DEBUG_LED_2_GPIO_Port GPIOA
#define QUAD_1_B_Pin GPIO_PIN_4
#define QUAD_1_B_GPIO_Port GPIOA
#define QUAD_1_A_Pin GPIO_PIN_6
#define QUAD_1_A_GPIO_Port GPIOA
#define LIMIT_1_0_Pin GPIO_PIN_4
#define LIMIT_1_0_GPIO_Port GPIOC
#define LIMIT_1_1_Pin GPIO_PIN_0
#define LIMIT_1_1_GPIO_Port GPIOB
#define LIMIT_1_2_Pin GPIO_PIN_1
#define LIMIT_1_2_GPIO_Port GPIOB
#define LIMIT_1_3_Pin GPIO_PIN_2
#define LIMIT_1_3_GPIO_Port GPIOB
#define MOTOR_0_PWM_Pin GPIO_PIN_14
#define MOTOR_0_PWM_GPIO_Port GPIOB
#define MOTOR_0_DIR_Pin GPIO_PIN_15
#define MOTOR_0_DIR_GPIO_Port GPIOB
#define MOTOR_1_DIR_Pin GPIO_PIN_6
#define MOTOR_1_DIR_GPIO_Port GPIOC
#define MOTOR_1_PWM_Pin GPIO_PIN_8
#define MOTOR_1_PWM_GPIO_Port GPIOA
#define CAN_STANDBY_Pin GPIO_PIN_15
#define CAN_STANDBY_GPIO_Port GPIOA
#define QUAD_0_A_Pin GPIO_PIN_6
#define QUAD_0_A_GPIO_Port GPIOB
#define QUAD_0_B_Pin GPIO_PIN_7
#define QUAD_0_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
