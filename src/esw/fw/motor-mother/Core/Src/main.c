/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "i2c_bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_MOTORS 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

Pin *hbridge_forward_pins[NUM_MOTORS] = { NULL };
Pin *hbridge_backward_pins[NUM_MOTORS] = { NULL };
HBridge *hbridges[NUM_MOTORS] = { NULL };

Pin *forward_limit_switch_pins[NUM_MOTORS] = { NULL };
Pin *backward_limit_switch_pins[NUM_MOTORS] = { NULL };

LimitSwitch *forward_limit_switches[NUM_MOTORS] = { NULL };
LimitSwitch *backward_limit_switches[NUM_MOTORS] = { NULL };

QuadEncoder *quad_encoders[NUM_MOTORS] = { NULL };

AbsEncoder *abs_encoders[NUM_MOTORS] = { NULL };

ClosedLoopControl *controls[NUM_MOTORS] = { NULL };

Motor *motors[NUM_MOTORS] = { NULL };
I2CBus *i2c_bus = NULL;

I2CBus *absolute_enc_i2c_bus = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16) {

		for (int i = 0; i < NUM_MOTORS; ++i) {
			tick_motor(motors[i]);
		}

		CH_tick(i2c_bus);

		for (size_t i = 0; i < NUM_MOTORS; ++i) {
			if (quad_encoders[i]->valid) {
				update_quad_encoder(quad_encoders[i]);
			}
			if (forward_limit_switches[i]->valid) {
				update_limit_switch(forward_limit_switches[i]);
			}
			if (backward_limit_switches[i]->valid) {
				update_limit_switch(backward_limit_switches[i]);
			}
			if (motors[i]->valid) {
				update_motor_target(motors[i]);
				update_motor_speed(motors[i]);
				update_motor_limit_switches(motors[i]);
			}
		}
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		HAL_I2C_Slave_Seq_Receive_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, 1, I2C_LAST_FRAME);
		i2c_bus->operation = UNKNOWN;
	} else {
		if (i2c_bus->motor_id < NUM_MOTORS) {
			CH_prepare_send(i2c_bus, motors[i2c_bus->motor_id]);
		}
		uint8_t bytes_to_send = CH_num_send(i2c_bus);
		if (bytes_to_send != 0) {
			HAL_I2C_Slave_Seq_Transmit_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, bytes_to_send, I2C_LAST_FRAME);
		}
	}

	i2c_bus->tick = 0;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (i2c_bus->operation == UNKNOWN) {
		i2c_bus->motor_id = (i2c_bus->buffer[0] >> 5) & 0x07;
		i2c_bus->operation = i2c_bus->buffer[0] & 0x1F;
		uint8_t bytes_to_recieve = CH_num_receive(i2c_bus);
		if (bytes_to_recieve != 0) {
			HAL_I2C_Slave_Seq_Receive_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, bytes_to_recieve, I2C_LAST_FRAME);
		} else {
			if (i2c_bus->motor_id < NUM_MOTORS) {
				CH_process_received(i2c_bus, motors[i2c_bus->motor_id]);
			}
		}
	} else {
		if (i2c_bus->motor_id < NUM_MOTORS) {
			CH_process_received(i2c_bus, motors[i2c_bus->motor_id]);
		}
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	CH_reset(i2c_bus);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(i2c_bus->i2c_bus_handle);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	hbridge_forward_pins[0] = new_pin(MOTOR_DIR_0_GPIO_Port, MOTOR_DIR_0_Pin);
	hbridge_forward_pins[1] = new_pin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin);
	hbridge_forward_pins[2] = new_pin(MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin);
	hbridge_forward_pins[3] = new_pin(MOTOR_DIR_3_GPIO_Port, MOTOR_DIR_3_Pin);
	hbridge_forward_pins[4] = new_pin(MOTOR_DIR_4_GPIO_Port, MOTOR_DIR_4_Pin);
	hbridge_forward_pins[5] = new_pin(MOTOR_DIR_5_GPIO_Port, MOTOR_DIR_5_Pin);

	hbridge_backward_pins[0] = new_pin(MOTOR_NDIR_0_GPIO_Port, MOTOR_NDIR_0_Pin);
	hbridge_backward_pins[1] = new_pin(MOTOR_NDIR_1_GPIO_Port, MOTOR_NDIR_1_Pin);
	hbridge_backward_pins[2] = new_pin(MOTOR_NDIR_2_GPIO_Port, MOTOR_NDIR_2_Pin);
	hbridge_backward_pins[3] = new_pin(MOTOR_NDIR_3_GPIO_Port, MOTOR_NDIR_3_Pin);
	hbridge_backward_pins[4] = new_pin(MOTOR_NDIR_4_GPIO_Port, MOTOR_NDIR_4_Pin);
	hbridge_backward_pins[5] = new_pin(MOTOR_NDIR_5_GPIO_Port, MOTOR_NDIR_5_Pin);

	hbridges[0] = new_hbridge(1, &htim4, TIM_CHANNEL_1, &(TIM4->CCR1), &(TIM4->ARR), hbridge_forward_pins[0], hbridge_backward_pins[0]);
	hbridges[1] = new_hbridge(1, &htim4, TIM_CHANNEL_2, &(TIM4->CCR2), &(TIM4->ARR), hbridge_forward_pins[1], hbridge_backward_pins[1]);
	hbridges[2] = new_hbridge(1, &htim4, TIM_CHANNEL_3, &(TIM4->CCR3), &(TIM4->ARR), hbridge_forward_pins[2], hbridge_backward_pins[2]);
	hbridges[3] = new_hbridge(1, &htim4, TIM_CHANNEL_4, &(TIM4->CCR4), &(TIM4->ARR), hbridge_forward_pins[3], hbridge_backward_pins[3]);
	hbridges[4] = new_hbridge(1, &htim15, TIM_CHANNEL_1, &(TIM15->CCR1), &(TIM15->ARR), hbridge_forward_pins[4], hbridge_backward_pins[4]);
	hbridges[5] = new_hbridge(1, &htim15, TIM_CHANNEL_2, &(TIM15->CCR2), &(TIM15->ARR), hbridge_forward_pins[5], hbridge_backward_pins[5]);

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		if (hbridges[i]->valid) {
			init_hbridge(hbridges[i], 0.0f, true);
		}
	}

	forward_limit_switch_pins[0] = new_pin(LIMIT_A_0_GPIO_Port, LIMIT_A_0_Pin);
	forward_limit_switch_pins[1] = new_pin(LIMIT_A_1_GPIO_Port, LIMIT_A_1_Pin);
	forward_limit_switch_pins[2] = new_pin(LIMIT_A_2_GPIO_Port, LIMIT_A_2_Pin);
	forward_limit_switch_pins[3] = new_pin(LIMIT_A_3_GPIO_Port, LIMIT_A_3_Pin);
	forward_limit_switch_pins[4] = new_pin(LIMIT_A_4_GPIO_Port, LIMIT_A_4_Pin);
	forward_limit_switch_pins[5] = new_pin(LIMIT_A_5_GPIO_Port, LIMIT_A_5_Pin);

	backward_limit_switch_pins[0] = new_pin(LIMIT_B_0_GPIO_Port, LIMIT_B_0_Pin);
	backward_limit_switch_pins[1] = new_pin(LIMIT_B_1_GPIO_Port, LIMIT_B_1_Pin);
	backward_limit_switch_pins[2] = new_pin(LIMIT_B_2_GPIO_Port, LIMIT_B_2_Pin);
	backward_limit_switch_pins[3] = new_pin(LIMIT_B_3_GPIO_Port, LIMIT_B_3_Pin);
	backward_limit_switch_pins[4] = new_pin(LIMIT_B_4_GPIO_Port, LIMIT_B_4_Pin);
	backward_limit_switch_pins[5] = new_pin(LIMIT_B_5_GPIO_Port, LIMIT_B_5_Pin);

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		forward_limit_switches[i] = new_limit_switch(
				forward_limit_switch_pins[i] != NULL,
				forward_limit_switch_pins[i]);
		backward_limit_switches[i] = new_limit_switch(
				backward_limit_switch_pins[i] != NULL,
				backward_limit_switch_pins[i]);
	}

	quad_encoders[0] = new_quad_encoder(1, &htim1, TIM1);
	quad_encoders[1] = new_quad_encoder(1, &htim2, TIM2);
	quad_encoders[2] = new_quad_encoder(1, &htim3, TIM3);

	// THE OTHER QUAD ENCODERS DO NOT EXIST. Only first value matters.
	quad_encoders[3] = new_quad_encoder(0, &htim1, TIM1);
	quad_encoders[4] = new_quad_encoder(0, &htim1, TIM1);
	quad_encoders[5] = new_quad_encoder(0, &htim1, TIM1);

	abs_encoders[0] = abs_encoder_init(1, &hi2c2, FALSE, FALSE);
	abs_encoders[1] = abs_encoder_init(1, &hi2c2, TRUE, TRUE);

	// THE OTHER ABS ENCODERS DO NOT EXIST. Only first value matters.
	abs_encoders[2] = abs_encoder_init(0, &hi2c2, FALSE, FALSE);
	abs_encoders[3] = abs_encoder_init(0, &hi2c2, FALSE, FALSE);
	abs_encoders[4] = abs_encoder_init(0, &hi2c2, FALSE, FALSE);
	abs_encoders[5] = abs_encoder_init(0, &hi2c2, FALSE, FALSE);

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		if (quad_encoders[i]->valid) {
			init_quad_encoder(quad_encoders[i]);
		}
	}

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		controls[i] = new_closed_loop_control(0.01f, 0.0f, 0.0f, 0.0f);
	}

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		motors[i] = new_motor(1, hbridges[i], forward_limit_switches[i], backward_limit_switches[i], quad_encoders[i], abs_encoders[i], controls[i]);
		init_motor(motors[i], 0.0f);
	}

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_I2C_MspInit(&hi2c1);

	i2c_bus = new_i2c_bus(&hi2c1); // NOTE: hi2c1 orig
	absolute_enc_i2c_bus = new_i2c_bus(&hi2c1);

	// TODO - Make this better so you wouldn't have to update on both sides (make a variable/class)

	// Start up the H-Bridge PWMs
	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		if (hbridges[i]->valid) {
			HAL_TIM_PWM_Start(hbridges[i]->timer, hbridges[i]->channel);
		}
	}

	// Start up the quadrature encoders
	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		if (quad_encoders[i]->valid) {
			HAL_TIM_Encoder_Start(quad_encoders[i]->htim, TIM_CHANNEL_ALL);
		}
	}

	// Start the logic loop timer
	HAL_TIM_Base_Start_IT(&htim16);

	// Start the I2C interrupts
	HAL_I2C_EnableListen_IT(i2c_bus->i2c_bus_handle);
//    HAL_I2C_EnableListen_IT(&hi2c1); // NOTE: hi2c1 orig

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	if (I2C_ADDRESS == 0x10)
	{
		while (1) {
			refresh_motor_absolute_encoder_value(motors[0]);
		}
	}

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 4;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = QUADRATURE_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = QUADRATURE_FILTER;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = QUADRATURE_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = QUADRATURE_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = QUADRATURE_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = QUADRATURE_FILTER;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 3;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 100;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = COUNTER_LOOP_PERIOD;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_DIR_4_Pin|MOTOR_NDIR_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_5_Pin|MOTOR_NDIR_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_DIR_0_Pin|MOTOR_NDIR_0_Pin|MOTOR_DIR_1_Pin|MOTOR_NDIR_1_Pin
                          |DEBUG_LED_1_Pin|DEBUG_LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_2_Pin|MOTOR_NDIR_2_Pin|MOTOR_DIR_3_Pin|MOTOR_NDIR_3_Pin
                          |DEBUG_LED_3_Pin|DEBUG_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_DIR_4_Pin MOTOR_NDIR_4_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_4_Pin|MOTOR_NDIR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_5_Pin MOTOR_NDIR_5_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_5_Pin|MOTOR_NDIR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_A_4_Pin LIMIT_B_4_Pin */
  GPIO_InitStruct.Pin = LIMIT_A_4_Pin|LIMIT_B_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_A_5_Pin */
  GPIO_InitStruct.Pin = LIMIT_A_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIMIT_A_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_B_5_Pin */
  GPIO_InitStruct.Pin = LIMIT_B_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT_B_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_0_Pin MOTOR_NDIR_0_Pin MOTOR_DIR_1_Pin MOTOR_NDIR_1_Pin
                           DEBUG_LED_1_Pin DEBUG_LED_0_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_0_Pin|MOTOR_NDIR_0_Pin|MOTOR_DIR_1_Pin|MOTOR_NDIR_1_Pin
                          |DEBUG_LED_1_Pin|DEBUG_LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_2_Pin MOTOR_NDIR_2_Pin MOTOR_DIR_3_Pin MOTOR_NDIR_3_Pin
                           DEBUG_LED_3_Pin DEBUG_LED_2_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_2_Pin|MOTOR_NDIR_2_Pin|MOTOR_DIR_3_Pin|MOTOR_NDIR_3_Pin
                          |DEBUG_LED_3_Pin|DEBUG_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_A_0_Pin LIMIT_B_0_Pin LIMIT_A_1_Pin LIMIT_B_1_Pin */
  GPIO_InitStruct.Pin = LIMIT_A_0_Pin|LIMIT_B_0_Pin|LIMIT_A_1_Pin|LIMIT_B_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_A_2_Pin LIMIT_B_2_Pin LIMIT_A_3_Pin LIMIT_B_3_Pin */
  GPIO_InitStruct.Pin = LIMIT_A_2_Pin|LIMIT_B_2_Pin|LIMIT_A_3_Pin|LIMIT_B_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
