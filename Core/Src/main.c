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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define receiving 1
uint8_t I2C_is_receiving_speed = 0;
uint8_t I2C_is_receiving_temp = 0;
uint8_t LED_State = 1;

#define slave_addr 0x01
uint16_t data_size = 1;
uint8_t I2C_status = HAL_OK;

#define connected_to_temp -120
int8_t is_connected_to_temp = 0;
int8_t data_temp_label = -120;
uint8_t data_temp = 11;
uint8_t polling = 0;

#define connected_to_speed -119
int8_t is_connected_to_speed = 0;
int8_t data_speed_label = -119;
uint8_t data_speed = 5;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim14)
	{
		HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, &data_temp_label, data_size);
		HAL_GPIO_WritePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin, LED_State);
		LED_State = !LED_State;
		I2C_is_receiving_temp = 1;
	}
}
*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  CAN_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  for(uint8_t i = 0; i < 10; i++)
  {
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
	  HAL_Delay(50);
  }

/*
  HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &I2C_status, data_size, HAL_MAX_DELAY);
  //SLAVE MUSI MIEĆ ZASILANIE W MOMENCIE WYWO�?ANIA TEJ FUNKCJI, INACZEJ PROGRAM PÓJDZIE DALEJ
  HAL_Delay(200);
  HAL_TIM_Base_Start_IT(&htim14);
*/

  while (1)
  {
		  //HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
		 // HAL_Delay(50);



/*
		if(I2C_is_receiving_temp == receiving)
		{
			while(is_connected_to_temp != connected_to_temp)
			{
			HAL_I2C_Master_Receive(&hi2c1, slave_addr, &is_connected_to_temp, data_size, HAL_MAX_DELAY);
			}

			HAL_Delay(5); // Konieczne opóźnienie aby slave zdążył przejść do nasłuchu
			HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, &data_temp, data_size);

			HAL_Delay(10); // Konieczne opóźnienie aby slave zdążył przejść do nasłuchu

			data_temp++;
			is_connected_to_temp = 0;
			I2C_is_receiving_temp = 0;
		}
*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
