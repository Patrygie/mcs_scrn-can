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
#include "stm32f0xx_hal.h"

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
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_2
#define SW3_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_3
#define SW4_GPIO_Port GPIOA
#define SW5_Pin GPIO_PIN_4
#define SW5_GPIO_Port GPIOA
#define SW6_Pin GPIO_PIN_5
#define SW6_GPIO_Port GPIOA
#define SW7_Pin GPIO_PIN_6
#define SW7_GPIO_Port GPIOA
#define SW8_Pin GPIO_PIN_7
#define SW8_GPIO_Port GPIOA
#define SW9_Pin GPIO_PIN_0
#define SW9_GPIO_Port GPIOB
#define SW10_Pin GPIO_PIN_1
#define SW10_GPIO_Port GPIOB
#define PCB_LED_RED_Pin GPIO_PIN_10
#define PCB_LED_RED_GPIO_Port GPIOB
#define PCB_LED_GREEN_Pin GPIO_PIN_11
#define PCB_LED_GREEN_GPIO_Port GPIOB
#define LED_MF_GREEN_Pin GPIO_PIN_12
#define LED_MF_GREEN_GPIO_Port GPIOB
#define LED_MF_RED_Pin GPIO_PIN_13
#define LED_MF_RED_GPIO_Port GPIOB
#define LED_MDF_GREEN_Pin GPIO_PIN_14
#define LED_MDF_GREEN_GPIO_Port GPIOB
#define LED_MDF_RED_Pin GPIO_PIN_15
#define LED_MDF_RED_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_9
#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_10
#define I2C_SDA_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
