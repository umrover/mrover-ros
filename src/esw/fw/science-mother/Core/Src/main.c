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

#include "adc_sensor.h"
#include "bridge.h"
#include "heater.h"
#include "pin_data.h"
#include "servo.h"
#include "smbus.h"
#include "spectral.h"
#include "thermistor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_MOSFET_DEVICES 12
#define NUM_SCIENCE_TEMP_SENSORS 3
#define NUM_SERVOS 3
#define NUM_HEATERS 3
#define NUM_DEBUG_LEDS 4

#define HEATER_0_MOSFET_PIN 10
#define HEATER_1_MOSFET_PIN 4
#define HEATER_2_MOSFET_PIN 5

#define AUTON_LED_RED_MOSFET_PIN 7
#define AUTON_LED_GREEN_MOSFET_PIN 8
#define AUTON_LED_BLUE_MOSFET_PIN 9

#define SERVO_0_DEFAULT_ANGLE 100
#define SERVO_1_DEFAULT_ANGLE 90  // TODO - change eventually too
#define SERVO_2_DEFAULT_ANGLE 80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

bool UART_watchdog = false;
AutonLED* auton_led = NULL;
ADCSensor* adc_sensor = NULL;
Bridge* bridge = NULL;
Heater* science_heaters[NUM_HEATERS] = {NULL};
PinData* debug_leds[NUM_DEBUG_LEDS] = {NULL};
PinData* heater_pins[NUM_HEATERS] = {NULL};
PinData* mosfet_pins[NUM_MOSFET_DEVICES] = {NULL};
Servo* servos[NUM_SERVOS] = {NULL};
SMBus* smbus = NULL;
Spectral* spectral = NULL;
Thermistor* science_temp_sensors[NUM_SCIENCE_TEMP_SENSORS] = {NULL};

float science_temperatures[NUM_SCIENCE_TEMP_SENSORS] = {0};
uint16_t spectral_data[SPECTRAL_CHANNELS] = {0};
bool heater_auto_shutoff_state = false;
bool heater_on_state[NUM_HEATERS] = {false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		update_auton_led_state(auton_led);
		UART_CH_tick(bridge);
		for (size_t i = 0; i < NUM_HEATERS; ++i) {
			tick_heater(science_heaters[i]);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	receive_bridge(bridge, science_heaters, mosfet_pins, servos, auton_led);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Delay so Jetson doen't kill it with noise

  // Temporarily disable USART to be resistant to noise (does this even work?)
  // 0x18 is the address of the RCC_APB2ENR register
//  CLEAR_BIT(0x18, RCC_APB2ENR_USART1EN);
  //HAL_Delay(60000);
//  SET_BIT(0x18, RCC_APB2ENR_USART1EN);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	adc_sensor = new_adc_sensor(&hadc1, 3);

	bridge = new_bridge(&huart1);

	debug_leds[0] = new_pin_data(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin, PIN_IS_OUTPUT);
	debug_leds[1] = new_pin_data(DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin, PIN_IS_OUTPUT);
	debug_leds[2] = new_pin_data(DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin, PIN_IS_OUTPUT);
	debug_leds[3] = new_pin_data(DEBUG_LED_3_GPIO_Port, DEBUG_LED_3_Pin, PIN_IS_OUTPUT);

	mosfet_pins[0] = new_pin_data(MOSFET_0_GPIO_Port, MOSFET_0_Pin, PIN_IS_OUTPUT);
	mosfet_pins[1] = new_pin_data(MOSFET_1_GPIO_Port, MOSFET_1_Pin, PIN_IS_OUTPUT);
	mosfet_pins[2] = new_pin_data(MOSFET_2_GPIO_Port, MOSFET_2_Pin, PIN_IS_OUTPUT);
	mosfet_pins[3] = new_pin_data(MOSFET_3_GPIO_Port, MOSFET_3_Pin, PIN_IS_OUTPUT);
	mosfet_pins[4] = new_pin_data(MOSFET_4_GPIO_Port, MOSFET_4_Pin, PIN_IS_OUTPUT);
	mosfet_pins[5] = new_pin_data(MOSFET_5_GPIO_Port, MOSFET_5_Pin, PIN_IS_OUTPUT);
	mosfet_pins[6] = new_pin_data(MOSFET_6_GPIO_Port, MOSFET_6_Pin, PIN_IS_OUTPUT);
	mosfet_pins[7] = new_pin_data(MOSFET_7_GPIO_Port, MOSFET_7_Pin, PIN_IS_OUTPUT);
	mosfet_pins[8] = new_pin_data(MOSFET_8_GPIO_Port, MOSFET_8_Pin, PIN_IS_OUTPUT);
	mosfet_pins[9] = new_pin_data(MOSFET_9_GPIO_Port, MOSFET_9_Pin, PIN_IS_OUTPUT);
	mosfet_pins[10] = new_pin_data(MOSFET_10_GPIO_Port, MOSFET_10_Pin, PIN_IS_OUTPUT);
	mosfet_pins[11] = new_pin_data(MOSFET_11_GPIO_Port, MOSFET_11_Pin, PIN_IS_OUTPUT);

	auton_led = new_auton_led(mosfet_pins[AUTON_LED_RED_MOSFET_PIN], mosfet_pins[AUTON_LED_GREEN_MOSFET_PIN], mosfet_pins[AUTON_LED_BLUE_MOSFET_PIN]);

	heater_pins[0] = mosfet_pins[HEATER_0_MOSFET_PIN];
	heater_pins[1] = mosfet_pins[HEATER_1_MOSFET_PIN];
	heater_pins[2] = mosfet_pins[HEATER_2_MOSFET_PIN];

	servos[0] = new_servo(&htim1, TIM_CHANNEL_1, &(TIM1->CCR1));
	servos[1] = new_servo(&htim1, TIM_CHANNEL_2, &(TIM1->CCR2));
	servos[2] = new_servo(&htim1, TIM_CHANNEL_3, &(TIM1->CCR3));

	smbus = new_smbus(&hi2c1, NULL, false);
	spectral = new_spectral(smbus);

	science_temp_sensors[0] = new_thermistor(adc_sensor, 0);
	science_temp_sensors[1] = new_thermistor(adc_sensor, 1);
	science_temp_sensors[2] = new_thermistor(adc_sensor, 2);

	science_heaters[0] = new_heater(heater_pins[0], science_temp_sensors[0]);
	science_heaters[1] = new_heater(heater_pins[1], science_temp_sensors[1]);
	science_heaters[2] = new_heater(heater_pins[2], science_temp_sensors[2]);

	receive_bridge(bridge, science_heaters, mosfet_pins, servos, auton_led);

  initialize_servo(servos[0], SERVO_0_DEFAULT_ANGLE);
  initialize_servo(servos[1], SERVO_1_DEFAULT_ANGLE);
  initialize_servo(servos[2], SERVO_2_DEFAULT_ANGLE);
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*
	   * In a while loop, we will need to get the following data and send it over:
	   * Science Temperatures
	   * Spectral sensor data
	   * State of auto-shut off state of the heater (upon change).
	   * State of the heater (upon change).
	   */
	  update_adc_sensor_values(adc_sensor);

	  for (size_t i = 0; i < NUM_HEATERS; ++i) {
		  update_heater_temperature(science_heaters[i]);
		  update_heater_state(science_heaters[i]);
		  science_temperatures[i] = get_thermistor_temperature(science_temp_sensors[i]);
	  }
	  bridge_send_science_thermistors(bridge, science_temperatures);

	  // Initialize spectral (in case it disconnected before and needs to reconnect)
	  initialize_spectral(spectral);

	  for (size_t i = 0; i < SPECTRAL_CHANNELS; ++i) {
		  uint8_t error_flag = 0;
		  update_spectral_channel_data(spectral, i, &error_flag);

		  // If spectral I2C NAKs, leave early
		  if(error_flag) {
			  break;
		  }

		  spectral_data[i] = get_spectral_channel_data(spectral, i);
	   }
	   bridge_send_spectral(bridge, spectral_data);

	   bool send_auto_shutoff = false;
	   for (size_t i = 0; i < NUM_HEATERS; ++i) {
		   heater_auto_shutoff_state = science_heaters[i]->auto_shutoff;
		   if (science_heaters[i]->send_auto_shutoff) {
			   science_heaters[i]->send_auto_shutoff = false;
			   send_auto_shutoff = true;
		   }
	   }
	   if (send_auto_shutoff) {
		   bridge_send_heater_auto_shutoff(bridge, heater_auto_shutoff_state);
	   }

	   bool send_heater_on = false;
	   for (size_t i = 0; i < NUM_HEATERS; ++i) {
		   heater_on_state[i] = science_heaters[i]->is_on;
		   if (science_heaters[i]->send_on) {
			   science_heaters[i]->send_on = false;
			   send_heater_on = true;
		   }
	   }
	   if (send_heater_on) {
		   bridge_send_heater_state(bridge, heater_on_state);
	   }
	   if (bridge->UART_watchdog_flag){
		   UART_CH_reset(bridge);
	   }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = TIMER_ONE_MS;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOSFET_0_Pin|MOSFET_1_Pin|MOSFET_2_Pin|MOSFET_3_Pin
                          |DEBUG_LED_1_Pin|DEBUG_LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOSFET_4_Pin|MOSFET_5_Pin|MOSFET_6_Pin|MOSFET_7_Pin
                          |MOSFET_8_Pin|MOSFET_9_Pin|MOSFET_11_Pin|MOSFET_10_Pin
                          |DEBUG_LED_3_Pin|DEBUG_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOSFET_0_Pin MOSFET_1_Pin MOSFET_2_Pin MOSFET_3_Pin
                           DEBUG_LED_1_Pin DEBUG_LED_0_Pin */
  GPIO_InitStruct.Pin = MOSFET_0_Pin|MOSFET_1_Pin|MOSFET_2_Pin|MOSFET_3_Pin
                          |DEBUG_LED_1_Pin|DEBUG_LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSFET_4_Pin MOSFET_5_Pin MOSFET_6_Pin MOSFET_7_Pin
                           MOSFET_8_Pin MOSFET_9_Pin MOSFET_11_Pin MOSFET_10_Pin
                           DEBUG_LED_3_Pin DEBUG_LED_2_Pin */
  GPIO_InitStruct.Pin = MOSFET_4_Pin|MOSFET_5_Pin|MOSFET_6_Pin|MOSFET_7_Pin
                          |MOSFET_8_Pin|MOSFET_9_Pin|MOSFET_11_Pin|MOSFET_10_Pin
                          |DEBUG_LED_3_Pin|DEBUG_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  while (1)
  {
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
