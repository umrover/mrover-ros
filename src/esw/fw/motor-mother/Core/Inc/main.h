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
#define I2C_ADDRESS 0x02
#define COUNTER_LOOP_PERIOD 1000
#define QUADRATURE_FILTER 8
#define QUAD_A_1_Pin GPIO_PIN_0
#define QUAD_A_1_GPIO_Port GPIOA
#define QUAD_B_1_Pin GPIO_PIN_1
#define QUAD_B_1_GPIO_Port GPIOA
#define MOTOR_PWM_4_Pin GPIO_PIN_2
#define MOTOR_PWM_4_GPIO_Port GPIOA
#define MOTOR_PWM_5_Pin GPIO_PIN_3
#define MOTOR_PWM_5_GPIO_Port GPIOA
#define MOTOR_DIR_4_Pin GPIO_PIN_4
#define MOTOR_DIR_4_GPIO_Port GPIOA
#define MOTOR_NDIR_4_Pin GPIO_PIN_5
#define MOTOR_NDIR_4_GPIO_Port GPIOA
#define QUAD_A_2_Pin GPIO_PIN_6
#define QUAD_A_2_GPIO_Port GPIOA
#define QUAD_B_2_Pin GPIO_PIN_7
#define QUAD_B_2_GPIO_Port GPIOA
#define MOTOR_DIR_5_Pin GPIO_PIN_4
#define MOTOR_DIR_5_GPIO_Port GPIOC
#define MOTOR_NDIR_5_Pin GPIO_PIN_5
#define MOTOR_NDIR_5_GPIO_Port GPIOC
#define LIMIT_A_4_Pin GPIO_PIN_0
#define LIMIT_A_4_GPIO_Port GPIOB
#define LIMIT_B_4_Pin GPIO_PIN_1
#define LIMIT_B_4_GPIO_Port GPIOB
#define LIMIT_A_5_Pin GPIO_PIN_2
#define LIMIT_A_5_GPIO_Port GPIOB
#define LIMIT_B_5_Pin GPIO_PIN_7
#define LIMIT_B_5_GPIO_Port GPIOE
#define QUAD_A_0_Pin GPIO_PIN_9
#define QUAD_A_0_GPIO_Port GPIOE
#define QUAD_B_0_Pin GPIO_PIN_11
#define QUAD_B_0_GPIO_Port GPIOE
#define MOTOR_DIR_0_Pin GPIO_PIN_12
#define MOTOR_DIR_0_GPIO_Port GPIOB
#define MOTOR_NDIR_0_Pin GPIO_PIN_13
#define MOTOR_NDIR_0_GPIO_Port GPIOB
#define MOTOR_DIR_1_Pin GPIO_PIN_14
#define MOTOR_DIR_1_GPIO_Port GPIOB
#define MOTOR_NDIR_1_Pin GPIO_PIN_15
#define MOTOR_NDIR_1_GPIO_Port GPIOB
#define MOTOR_DIR_2_Pin GPIO_PIN_8
#define MOTOR_DIR_2_GPIO_Port GPIOD
#define MOTOR_NDIR_2_Pin GPIO_PIN_9
#define MOTOR_NDIR_2_GPIO_Port GPIOD
#define MOTOR_DIR_3_Pin GPIO_PIN_10
#define MOTOR_DIR_3_GPIO_Port GPIOD
#define MOTOR_NDIR_3_Pin GPIO_PIN_11
#define MOTOR_NDIR_3_GPIO_Port GPIOD
#define MOTOR_PWM_0_Pin GPIO_PIN_12
#define MOTOR_PWM_0_GPIO_Port GPIOD
#define MOTOR_PWM_1_Pin GPIO_PIN_13
#define MOTOR_PWM_1_GPIO_Port GPIOD
#define MOTOR_PWM_2_Pin GPIO_PIN_14
#define MOTOR_PWM_2_GPIO_Port GPIOD
#define MOTOR_PWM_3_Pin GPIO_PIN_15
#define MOTOR_PWM_3_GPIO_Port GPIOD
#define LIMIT_A_0_Pin GPIO_PIN_6
#define LIMIT_A_0_GPIO_Port GPIOC
#define LIMIT_B_0_Pin GPIO_PIN_7
#define LIMIT_B_0_GPIO_Port GPIOC
#define LIMIT_A_1_Pin GPIO_PIN_8
#define LIMIT_A_1_GPIO_Port GPIOC
#define LIMIT_B_1_Pin GPIO_PIN_9
#define LIMIT_B_1_GPIO_Port GPIOC
#define LIMIT_A_2_Pin GPIO_PIN_8
#define LIMIT_A_2_GPIO_Port GPIOA
#define LIMIT_B_2_Pin GPIO_PIN_9
#define LIMIT_B_2_GPIO_Port GPIOA
#define LIMIT_A_3_Pin GPIO_PIN_10
#define LIMIT_A_3_GPIO_Port GPIOA
#define LIMIT_B_3_Pin GPIO_PIN_11
#define LIMIT_B_3_GPIO_Port GPIOA
#define DEBUG_LED_3_Pin GPIO_PIN_6
#define DEBUG_LED_3_GPIO_Port GPIOD
#define DEBUG_LED_2_Pin GPIO_PIN_7
#define DEBUG_LED_2_GPIO_Port GPIOD
#define DEBUG_LED_1_Pin GPIO_PIN_3
#define DEBUG_LED_1_GPIO_Port GPIOB
#define DEBUG_LED_0_Pin GPIO_PIN_4
#define DEBUG_LED_0_GPIO_Port GPIOB
#define I2C1_TO_JETSON_SCL_Pin GPIO_PIN_8
#define I2C1_TO_JETSON_SCL_GPIO_Port GPIOB
#define I2C1_TO_JETSON_SDA_Pin GPIO_PIN_9
#define I2C1_TO_JETSON_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
