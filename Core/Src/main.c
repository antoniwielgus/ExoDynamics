/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "can_handler.h"
#include "AK80_64_driver.h"

#include <stdio.h>
#include <string.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CanHandler motor1;
uint8_t can_id = 0;

uint8_t serial_received_data[23];

union
{
  uint8_t char_array[4];
  float float_value;
} char_array_to_float;

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
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  initCanHandler(&motor1, 8, can_id, CAN_ID_STD, CAN_RTR_DATA, can_id, DISABLE);
  HAL_CAN_Start(&hcan1);

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_UART_Receive_IT(&huart2, serial_received_data, 23);

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
    

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    can_id = serial_received_data[0];
    char msg[20];
    sprintf(msg, "Can id: %d\r\n", can_id);
    HAL_UART_Transmit(&huart1, msg, 20, HAL_MAX_DELAY);

    float p_des, v_des, kp, kd, t_ff;

    // if serial_received_data[1] == 0x01 this means that motor is running
    if (serial_received_data[1] == 0x02)
    {
      int i = 2;
      int start = i;

      for (; i < start + 4; i++)
        char_array_to_float.char_array[i - start] = serial_received_data[i];
      start = i;
      p_des = char_array_to_float.float_value;

      for (; i < start + 4; i++)
        char_array_to_float.char_array[i - start] = serial_received_data[i];
      start = i;
      v_des = char_array_to_float.float_value;

      for (; i < start + 4; i++)
        char_array_to_float.char_array[i - start] = serial_received_data[i];
      start = i;
      kp = char_array_to_float.float_value;

      for (; i < start + 4; i++)
        char_array_to_float.char_array[i - start] = serial_received_data[i];
      start = i;
      kd = char_array_to_float.float_value;

      for (; i < start + 4; i++)
        char_array_to_float.char_array[i - start] = serial_received_data[i];
      start = i;
      t_ff = char_array_to_float.float_value;





      // char msg[64];
      // sprintf((char*)msg, "p: %d\n\r", p_des);
      // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
      

      uint8_t frame[8];
      set_motor_possition(frame, p_des, v_des, kp, kd, t_ff);
      sendCanFrame(&hcan1, can_id, &motor1, frame);

      // set_motor_possition(frame, 0.0f, 6.28f, 0.0f, 0.5f, 0.0f);
      // sendCanFrame(&hcan1, &motor1, frame);
    }
    else if (serial_received_data[1] == 0x01) // start motor
      startEngine(&hcan1, &motor1);
    else if (serial_received_data[1] == 0x00) // stop motor
      stopEngine(&hcan1, &motor1);

    HAL_UART_Receive_IT(&huart2, serial_received_data, 23);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  receiveCanFrame(hcan, &motor1);

  char msg[64];
  sprintf((char*)msg, "Received CAN id: %d, %d, %d, %d, %d, %d, %d, %d\r\n", motor1.rxData[0], motor1.rxData[1], motor1.rxData[2], motor1.rxData[3], motor1.rxData[4], motor1.rxData[5], motor1.rxData[6], motor1.rxData[7]);
  HAL_UART_Transmit(&huart1, (char*)msg, 64, HAL_MAX_DELAY);

  HAL_UART_Transmit(&huart5, motor1.rxData, 8, HAL_MAX_DELAY);
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
