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
#include "stm32f4xx_hal.h"

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
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_14
#define LED_B_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOC
#define USER_BUTTON_Pin GPIO_PIN_3
#define USER_BUTTON_GPIO_Port GPIOC
#define VSENSE_A_Pin GPIO_PIN_0
#define VSENSE_A_GPIO_Port GPIOA
#define VSENSE_B_Pin GPIO_PIN_1
#define VSENSE_B_GPIO_Port GPIOA
#define VSENSE_C_Pin GPIO_PIN_2
#define VSENSE_C_GPIO_Port GPIOA
#define VSENSE_DC_Pin GPIO_PIN_3
#define VSENSE_DC_GPIO_Port GPIOA
#define ISEN_A_Pin GPIO_PIN_4
#define ISEN_A_GPIO_Port GPIOA
#define ISEN_B_Pin GPIO_PIN_5
#define ISEN_B_GPIO_Port GPIOA
#define ISEN_C_Pin GPIO_PIN_6
#define ISEN_C_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_7
#define ADC_TEMP_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_4
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_5
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_0
#define GPIO3_GPIO_Port GPIOB
#define NFAULT_Pin GPIO_PIN_10
#define NFAULT_GPIO_Port GPIOB
#define EN_DRIVER_Pin GPIO_PIN_11
#define EN_DRIVER_GPIO_Port GPIOB
#define TERM_Pin GPIO_PIN_15
#define TERM_GPIO_Port GPIOA
#define CS_DRIVER_Pin GPIO_PIN_2
#define CS_DRIVER_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
