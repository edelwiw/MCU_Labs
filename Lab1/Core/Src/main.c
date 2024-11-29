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
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void display(uint8_t val);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t vall = 0;
  bool btn1 = 0;
  bool btn2 = 0;
  unsigned int timer1 = 0;
  unsigned int timer2 = 0;
  while (1)
  {
    /* USER CODE END WHILE */
//	  for(uint8_t i = 0; i < 255; i++){
//		  display(i);
//		  HAL_Delay(300);
//	  }
	  if(!HAL_GPIO_ReadPin(Bt1_GPIO_Port, Bt1_Pin) && btn1 && vall > 0){
		  HAL_GPIO_WritePin(BtLed1_GPIO_Port, BtLed1_Pin , 1);
		  timer1 = 0;
		  vall --;
//		  HAL_Delay(100);
//		  HAL_GPIO_WritePin(BtLed1_GPIO_Port, BtLed1_Pin , 0);

	  }
	  btn1 = HAL_GPIO_ReadPin(Bt1_GPIO_Port, Bt1_Pin);
	  if(!HAL_GPIO_ReadPin(Bt2_GPIO_Port, Bt2_Pin) && btn2 && vall < 255){
		  HAL_GPIO_WritePin(BtLed2_GPIO_Port, BtLed2_Pin , 1);
		  timer2 = 0;
		  vall ++;

	  }
	  btn2 = HAL_GPIO_ReadPin(Bt2_GPIO_Port, Bt2_Pin);

	  display(vall);

	  if(timer1 > 10000){
		  HAL_GPIO_WritePin(BtLed1_GPIO_Port, BtLed1_Pin , 0);
	  }
	  if(timer2 > 10000){
		  HAL_GPIO_WritePin(BtLed2_GPIO_Port, BtLed2_Pin , 0);
	  }

	  timer1++;
	  timer2++;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led1_Pin|Led2_Pin|Led3_Pin|Led4_Pin
                          |Led5_Pin|Led6_Pin|Led7_Pin|Led8_Pin
                          |BtLed2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BtLed1_GPIO_Port, BtLed1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Bt1_Pin */
  GPIO_InitStruct.Pin = Bt1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Bt1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led1_Pin Led2_Pin Led3_Pin Led4_Pin
                           Led5_Pin Led6_Pin Led7_Pin Led8_Pin
                           BtLed2_Pin */
  GPIO_InitStruct.Pin = Led1_Pin|Led2_Pin|Led3_Pin|Led4_Pin
                          |Led5_Pin|Led6_Pin|Led7_Pin|Led8_Pin
                          |BtLed2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BtLed1_Pin */
  GPIO_InitStruct.Pin = BtLed1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BtLed1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Bt2_Pin */
  GPIO_InitStruct.Pin = Bt2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Bt2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void display(uint8_t val){
	HAL_GPIO_WritePin(GPIOC, Led1_Pin, val & (1 << 7));
	HAL_GPIO_WritePin(GPIOC, Led2_Pin, val & (1 << 6));
	HAL_GPIO_WritePin(GPIOC, Led3_Pin, val & (1 << 5));
	HAL_GPIO_WritePin(GPIOC, Led4_Pin, val & (1 << 4));
	HAL_GPIO_WritePin(GPIOC, Led5_Pin, val & (1 << 3));
	HAL_GPIO_WritePin(GPIOC, Led6_Pin, val & (1 << 2));
	HAL_GPIO_WritePin(GPIOC, Led7_Pin, val & (1 << 1));
	HAL_GPIO_WritePin(GPIOC, Led8_Pin, val & (1 << 0));
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
