/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
void SystemClock_Config(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RELAY_Pin GPIO_PIN_2
#define RELAY_GPIO_Port GPIOA
#define TERM_1_Pin GPIO_PIN_4
#define TERM_1_GPIO_Port GPIOA
#define TERM_2_Pin GPIO_PIN_5
#define TERM_2_GPIO_Port GPIOA
#define TERM_3_Pin GPIO_PIN_6
#define TERM_3_GPIO_Port GPIOA
#define TERM_4_Pin GPIO_PIN_7
#define TERM_4_GPIO_Port GPIOA
#define TERM_5_Pin GPIO_PIN_4
#define TERM_5_GPIO_Port GPIOC
#define TERM_6_Pin GPIO_PIN_5
#define TERM_6_GPIO_Port GPIOC
#define SENSOR_Pin GPIO_PIN_1
#define SENSOR_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_10
#define LED_3_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_11
#define LED_4_GPIO_Port GPIOB
#define LED_DASH_Pin GPIO_PIN_12
#define LED_DASH_GPIO_Port GPIOB
#define SW_DIO_Pin GPIO_PIN_13
#define SW_DIO_GPIO_Port GPIOA
#define SW_CLK_Pin GPIO_PIN_14
#define SW_CLK_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
