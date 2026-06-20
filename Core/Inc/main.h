/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define MCPWM_CLOCK_HZ 168000000
#define MCPWM_DEADTIME_CLOCKS 20
#define MCPWM_TGRO_TIME MCPWM_PERIOD_CLOCKS-10
#define MCPWM_RCR 0
#define MCPWM_FREQ 20000
#define MCPWM_PERIOD_CLOCKS MCPWM_CLOCK_HZ/2/MCPWM_FREQ
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CUR_A_Pin GPIO_PIN_0
#define CUR_A_GPIO_Port GPIOA
#define CUR_B_Pin GPIO_PIN_1
#define CUR_B_GPIO_Port GPIOA
#define CUR_C_Pin GPIO_PIN_2
#define CUR_C_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_1
#define VBUS_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_12
#define SPI1_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
