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
#include <string.h>
#include <stdio.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
static const uint8_t IMU_ADDR = 0x19<<1; 	//| Use 8-bit address
static const uint8_t REG_CTRL_1 = 0x20; 	//| Control Register-1
static const uint8_t REG_CTRL_4 = 0x23; 	//| Control Register-4
static const uint8_t REG_X_L 	= 0x28; 	//| X-Axis LSB
static const uint8_t REG_X_H 	= 0x29;		//| X-Axis MSB
static const uint8_t REG_Y_L 	= 0x2A;		//| Y-Axis LSB
static const uint8_t REG_Y_H 	= 0x2B;		//| Y-Axis MSB
static const uint8_t REG_Z_L 	= 0x2C;		//| Z-Axis LSB
static const uint8_t REG_Z_H 	= 0x2D;		//| Z-Axis MSB

//Constants used to convert raw acceleration based on range
static const uint16_t ACC_RANGE_2G = 16384;
static const uint16_t ACC_RANGE_4G = 8192;
static const uint16_t ACC_RANGE_8G = 4096;
static const uint16_t ACC_RANGE_16G = 2048;

//Control register 4 values to change acceleration range
static const uint8_t CTRL_REG_4_ACCL_RANGE_16G             = 0x30; // Full scale = +/-16g
static const uint8_t CTRL_REG_4_ACCL_RANGE_8G              = 0x20; // Full scale = +/-8g
static const uint8_t CTRL_REG_4_ACCL_RANGE_4G              = 0x10; // Full scale = +/-4g
static const uint8_t CTRL_REG_4_ACCL_RANGE_2G              = 0x00; //Full scale = +/-2g

//Selected acceleration range
static const uint16_t SELECTED_ACC_RANGE = ACC_RANGE_2G;
static const uint8_t SELECTED_CTRL_REG_4_ACCL_RANGE = CTRL_REG_4_ACCL_RANGE_2G;

static const float GRAVITATIONAL_CONSTANT = 9.8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void print(char _out[]);
float convertRawAcceleration(uint8_t highReading, uint8_t lowReading);
void readRawAcceleration(
		uint8_t * ACC_X_H,
		uint8_t * ACC_X_L,
		uint8_t * ACC_Y_H,
		uint8_t * ACC_Y_L,
		uint8_t * ACC_Z_H,
		uint8_t * ACC_Z_L);
void updatePos(float * pos, float speed, float acc, float TIME_DIFF);
void updateSpeed(float * speed, float acc,float TIME_DIFF);
void getTimeValues(int previousTime, int * currentTime, float * timeDiff);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t ACC_X_L = {0};
	uint8_t ACC_X_H = {0};
	uint8_t ACC_Y_L = {0};
	uint8_t ACC_Y_H = {0};
	uint8_t ACC_Z_L = {0};
	uint8_t ACC_Z_H = {0};

	float ACC_X = 0;
	float ACC_Y = 0;
	float ACC_Z = 0;

	float X_POS = 0;
	float X_SPEED = 0;
	float Y_POS = 0;
	float Y_SPEED = 0;

	int PREVIOUS_TIME = 0;
	int CURRENT_TIME = 0;
	float TIME_DIFF = 0;
	char buffer_msg[13];

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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t buf[1] = {0b01110111};
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_CTRL_1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1000);

  uint8_t buf2[1] = {SELECTED_CTRL_REG_4_ACCL_RANGE};
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_CTRL_4, I2C_MEMADD_SIZE_8BIT, buf2, 1, 1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //Read raw acceleration
	  readRawAcceleration(&ACC_X_H, &ACC_X_L, &ACC_Y_H, &ACC_Y_L, &ACC_Z_H, &ACC_Z_L);


	  //Convert raw acceleration
	  ACC_X = convertRawAcceleration(ACC_X_H,ACC_X_L);
	  ACC_Y = convertRawAcceleration(ACC_Y_H,ACC_Y_L);
	  ACC_Z = convertRawAcceleration(ACC_Z_H,ACC_Z_L);

	  //Get time
	  getTimeValues(PREVIOUS_TIME,&CURRENT_TIME,&TIME_DIFF);

	  //Update position values
	  updatePos(&X_POS, X_SPEED,ACC_X,TIME_DIFF);
	  updatePos(&Y_POS, Y_SPEED,ACC_Y,TIME_DIFF);


	  //Update speed values
	  updateSpeed(&X_SPEED,ACC_X,TIME_DIFF);
	  updateSpeed(&Y_SPEED,ACC_Y,TIME_DIFF);

	  //Print values
	  sprintf(buffer_msg, "%fm/s2", ACC_X);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm/s2", ACC_Y);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm/s2", ACC_Z);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm", X_POS);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm", Y_POS);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm/s", X_SPEED);
	  print(buffer_msg);

	  sprintf(buffer_msg, "%fm/s", Y_SPEED);
	  print(buffer_msg);

	  print("-------------");

	  //Update clock
	  PREVIOUS_TIME = CURRENT_TIME;
	  HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void print(char _out[]){
 HAL_UART_Transmit(&hlpuart1, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(&hlpuart1, (uint8_t *) newline, 2, 10);
}

float convertRawAcceleration(uint8_t highReading, uint8_t lowReading){
	  int16_t RAW_ACC = highReading;
	  RAW_ACC <<= 8;
	  RAW_ACC |= lowReading;
	  return ((float)RAW_ACC / (float)SELECTED_ACC_RANGE)*GRAVITATIONAL_CONSTANT;
}

void readRawAcceleration(
		uint8_t * ACC_X_H,
		uint8_t * ACC_X_L,
		uint8_t * ACC_Y_H,
		uint8_t * ACC_Y_L,
		uint8_t * ACC_Z_H,
		uint8_t * ACC_Z_L){
		// Get acceleration in X-AXIS
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_X_H, I2C_MEMADD_SIZE_8BIT, ACC_X_H, 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_X_L, I2C_MEMADD_SIZE_8BIT, ACC_X_L, 1, 1000);


		// Get acceleration in Y-AXIS
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Y_H, I2C_MEMADD_SIZE_8BIT, ACC_Y_H, 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Y_L, I2C_MEMADD_SIZE_8BIT, ACC_Y_L, 1, 1000);


		// Get acceleration in Z-AXIS
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Z_H, I2C_MEMADD_SIZE_8BIT, ACC_Z_H, 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Z_L, I2C_MEMADD_SIZE_8BIT, ACC_Z_L, 1, 1000);
}

void updatePos(float * pos, float speed, float acc, float TIME_DIFF){
	*pos += speed*TIME_DIFF+0.5*acc*TIME_DIFF*TIME_DIFF;
}
void updateSpeed(float * speed, float acc,float TIME_DIFF){
	*speed += acc*TIME_DIFF;
}

void getTimeValues(int previousTime, int * currentTime, float * timeDiff){
	*currentTime = HAL_GetTick();
	*timeDiff = ((float)*currentTime-(float)previousTime)/1000;
}

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

