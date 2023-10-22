/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_DEVICE_ADDR 0x49
#define I2C_STATUS_REG 0x00
#define I2C_READ_REG 0x01
#define I2C_WRITE_REG 0x02

#define	I2C_AS72XX_SLAVE_STATUS_REG 0x00
#define	I2C_AS72XX_SLAVE_WRITE_REG 0x01
#define	I2C_AS72XX_SLAVE_READ_REG 0x02
#define	I2C_AS72XX_SLAVE_TX_VALID 0x02
#define	I2C_AS72XX_SLAVE_RX_VALID 0x01

#define	DEVICE_SLAVE_ADDRESS_READ 0x93
#define	DEVICE_SLAVE_ADDRESS_WRITE 0x92
#define	DEVICE_SLAVE_ADDRESS 0x49

// registers for spectral sensor
#define	RAW_VALUE_RGA_HIGH 0x08
#define	RAW_VALUE_RGA_LOW 0x09

#define	RAW_VALUE_SHB_HIGH 0x0A
#define	RAW_VALUE_SHB_LOW 0x0B

#define	RAW_VALUE_TIC_HIGH 0x0C
#define	RAW_VALUE_TIC_LOW 0x0D

#define	RAW_VALUE_UJD_HIGH 0x0E
#define	RAW_VALUE_UJD_LOW 0x0F

#define	RAW_VALUE_VKE_HIGH 0x10
#define	RAW_VALUE_VKE_LOW 0x11

#define	RAW_VALUE_WLF_HIGH 0x12
#define	RAW_VALUE_WLF_LOW 0x13
#define	AS7262_V_CAL 0x14
#define	AS7262_B_CAL 0x18
#define	AS7262_G_CAL 0x1C
#define	AS7262_Y_CAL 0x20
#define	AS7262_O_CAL 0x24
#define	AS7262_R_CAL 0x28

#define	CONTROL_SET_UP 0x04
#define	INT_TIME 0x05

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// read register
// this is lowest level function
void i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size) {
	I2C_HandleTypeDef *i2c = &hi2c2;

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(i2c, dev_addr << 1, mem_addr, 1, buf, size, 100);

	if (status != HAL_OK)
	{
		HAL_I2C_DeInit(i2c);
		HAL_Delay(5);
		HAL_I2C_Init(i2c);
	}
}

void i2c_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size) {
	I2C_HandleTypeDef *i2c = &hi2c2;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(i2c, dev_addr << 1, mem_addr, 1, buf, size, 100);

	if (status != HAL_OK)
	{
		HAL_I2C_DeInit(i2c);
		HAL_Delay(5);
		HAL_I2C_Init(i2c);
	}
}

//void smbus_reset(SMBus *smbus)
//{
//    HAL_I2C_DeInit(smbus->i2c);
//    HAL_I2C_Init(smbus->i2c);
//}

// REQUIRES: smbus is an SMBus object,
// reg is the command/register being written to,
// and data is the data being written to the register.
// MODIFIES: nothing
// EFFECTS: Writes one byte to the register.
void smbus_write_byte_data(uint8_t reg, uint8_t* buf, uint8_t data, uint8_t device_address)
{
	buf[0] = data;


	I2C_HandleTypeDef *i2c = &hi2c2;
	HAL_StatusTypeDef status;

	// TODO: doesn't work when trying to write to reg 0x01 (the write register)
	status = HAL_I2C_Mem_Write(i2c, device_address << 1, reg, 1, buf, 1, 100);
    HAL_Delay(10);
}

long smbus_read_byte_data(uint8_t reg, uint8_t* buf, uint8_t device_address)
{
    buf[0] = reg;

	I2C_HandleTypeDef *i2c = &hi2c2;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(i2c, device_address << 1, reg, 1, buf, 1, 100);

	status = HAL_I2C_Mem_Read(i2c, (device_address << 1) | 1, reg, 1, buf, 1, 100);

    HAL_Delay(10);
    return buf[0];
}


void virtual_write_spectral(uint8_t v_reg, uint8_t* buf, uint8_t data) {
    long v_status = 2;

	while(1) {
		v_status = smbus_read_byte_data(I2C_AS72XX_SLAVE_STATUS_REG, buf, DEVICE_SLAVE_ADDRESS);
		if ((v_status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}

	smbus_write_byte_data(I2C_AS72XX_SLAVE_WRITE_REG, buf, (v_reg | 0x80), DEVICE_SLAVE_ADDRESS);

	while(1) {
		v_status = smbus_read_byte_data(I2C_AS72XX_SLAVE_STATUS_REG, buf, DEVICE_SLAVE_ADDRESS);
		if ((v_status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	smbus_write_byte_data(I2C_AS72XX_SLAVE_WRITE_REG, buf, data, DEVICE_SLAVE_ADDRESS);
}


uint8_t virtual_read_spectral(uint8_t v_reg, uint8_t *error, uint8_t* buf) {
    // Taken from datasheet
    uint8_t has_error;
	long d;
	long v_status = 2;

	while(1) {
		v_status = smbus_read_byte_data(I2C_AS72XX_SLAVE_STATUS_REG, buf, DEVICE_SLAVE_ADDRESS);
		if ((v_status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}

	smbus_write_byte_data(I2C_AS72XX_SLAVE_WRITE_REG, buf, (v_reg | 0x80), DEVICE_SLAVE_ADDRESS);
//	smbus_write_byte_data(I2C_AS72XX_SLAVE_WRITE_REG, buf, v_reg, DEVICE_SLAVE_ADDRESS);

//	while(1) {
//		v_status = smbus_read_byte_data(I2C_AS72XX_SLAVE_STATUS_REG, buf, DEVICE_SLAVE_ADDRESS);
//
//		if ((v_status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
//			break;
//		}
//		HAL_Delay(5);
//	}

	HAL_Delay(1);
	d = smbus_read_byte_data(I2C_AS72XX_SLAVE_READ_REG, buf, DEVICE_SLAVE_ADDRESS);
	return d;
}



// must connect pin 8 on spectral to high so that it actually does I2C
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t channel_data[6] = {0,0,0,0,0,0};
	uint8_t buffer[50];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // set up

  uint8_t setup_buf[50];
  virtual_write_spectral(CONTROL_SET_UP, setup_buf, 0x28);  // runs twice to account for status miss
  HAL_Delay(50);
  virtual_write_spectral(CONTROL_SET_UP, setup_buf, 0x28);  // converts data bank to 2
  // Integration time is 0xFF * 2.8ms
  HAL_Delay(50);
  virtual_write_spectral(INT_TIME, setup_buf, 0xFF);
  virtual_write_spectral(CONTROL_SET_UP, setup_buf, 0x28);  // runs twice to account for status miss
  HAL_Delay(50);
  virtual_write_spectral(CONTROL_SET_UP, setup_buf, 0x28);  // converts data bank to 2
  // Integration time is 0xFF * 2.8ms
  HAL_Delay(50);
  virtual_write_spectral(INT_TIME, setup_buf, 0xFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	for (int i = 0; i < 6; ++i) {
		uint8_t error_flag;
		uint8_t START_REG = RAW_VALUE_RGA_HIGH;

		uint8_t msb = START_REG + i * 2;
		uint8_t lsb = START_REG + i * 2 + 1;
		uint16_t high = (virtual_read_spectral(msb, &error_flag, buffer) & 0xFF) << 8;
		channel_data[i] = high | (virtual_read_spectral(lsb, &error_flag, buffer) & 0xFF);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
