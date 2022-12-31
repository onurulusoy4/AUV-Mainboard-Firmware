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
#include "stm32h7xx_hal.h"

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
#define AUX_PWM2_Pin GPIO_PIN_0
#define AUX_PWM2_GPIO_Port GPIOA
#define BOARD_LED_Pin GPIO_PIN_1
#define BOARD_LED_GPIO_Port GPIOA
#define ADC_INPUT1_Pin GPIO_PIN_6
#define ADC_INPUT1_GPIO_Port GPIOA
#define ADC_INPUT2_Pin GPIO_PIN_4
#define ADC_INPUT2_GPIO_Port GPIOC
#define AUX_PWM1_Pin GPIO_PIN_13
#define AUX_PWM1_GPIO_Port GPIOE
#define SCL_2_Pin GPIO_PIN_10
#define SCL_2_GPIO_Port GPIOB
#define SDA_2_Pin GPIO_PIN_11
#define SDA_2_GPIO_Port GPIOB
#define AUX_PWM3_Pin GPIO_PIN_12
#define AUX_PWM3_GPIO_Port GPIOD
#define AUX_PWM3B5_Pin GPIO_PIN_5
#define AUX_PWM3B5_GPIO_Port GPIOB
#define SCL_4_Pin GPIO_PIN_6
#define SCL_4_GPIO_Port GPIOB
#define SDA_4_Pin GPIO_PIN_7
#define SDA_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
