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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERR_Pin GPIO_PIN_5
#define ERR_GPIO_Port GPIOE
#define WARN_Pin GPIO_PIN_6
#define WARN_GPIO_Port GPIOE
#define IP_Pin GPIO_PIN_13
#define IP_GPIO_Port GPIOC
#define RUN_Pin GPIO_PIN_14
#define RUN_GPIO_Port GPIOC
#define E6_Pin GPIO_PIN_15
#define E6_GPIO_Port GPIOC
#define D_IN1_Pin GPIO_PIN_2
#define D_IN1_GPIO_Port GPIOB
#define D_IN2_Pin GPIO_PIN_7
#define D_IN2_GPIO_Port GPIOE
#define RS485_EN_Pin GPIO_PIN_15
#define RS485_EN_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_0
#define SPI3_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
