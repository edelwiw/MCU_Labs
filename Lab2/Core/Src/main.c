#include "main.h"
#include "stdbool.h"

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


#define RX_BUFFER_SIZE 4
#define TX_BUFFER_SIZE 100
uint8_t buffer[RX_BUFFER_SIZE];
char txBuffer[TX_BUFFER_SIZE];


void display(uint8_t val){
	HAL_GPIO_WritePin(GPIOC, Led1_Pin, val & (1 << 0));
	HAL_GPIO_WritePin(GPIOC, Led2_Pin, val & (1 << 1));
	HAL_GPIO_WritePin(GPIOC, Led3_Pin, val & (1 << 2));
	HAL_GPIO_WritePin(GPIOC, Led4_Pin, val & (1 << 3));
	HAL_GPIO_WritePin(GPIOC, Led5_Pin, val & (1 << 4));
	HAL_GPIO_WritePin(GPIOC, Led6_Pin, val & (1 << 5));
	HAL_GPIO_WritePin(GPIOC, Led7_Pin, val & (1 << 6));
	HAL_GPIO_WritePin(GPIOC, Led8_Pin, val & (1 << 7));
}


// commands 
// 0d .. .. 0a
// 0xaa -- ping 
// 0xbb -- on 
// 0xcc -- off 
// 0xdd -- set leds

uint8_t val = 0;

void processCommand(uint8_t buffer[RX_BUFFER_SIZE])
{ 
  uint8_t command = buffer[1];
  uint8_t value1 = buffer[2];


  char* message = malloc(100);
  sprintf(message, "Command: %02X, Value1: %02X\r\n", command, value1);
  HAL_UART_Transmit(&huart2, message, strlen(message), 1000);

  switch(command){
    case 0xaa:
      sprintf(txBuffer, "Command: %02X, Value1: %02X, Value2: %02X\r\n", command, value1);
      HAL_UART_Transmit_IT(&huart2, txBuffer, strlen(txBuffer));
      break;
    case 0xbb:
      val = 0xff;
      break;
    case 0xcc:
      val = 0;
      break;
    case 0xdd:
      val = value1;
      break;
    case 0xee:
     sprintf(txBuffer, "LEDS: %d\r\n", (GPIOC->ODR & (0xFF << 4)) >> 4);
    HAL_UART_Transmit_IT(&huart2, txBuffer, strlen(txBuffer));
    default:
      HAL_UART_Transmit_IT(&huart2, "Invalid command\r\n", 17);
      break; 
  }
  display(val);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();


  HAL_UART_Receive_IT(&huart2, &buffer, RX_BUFFER_SIZE); // start receiving data

  while (1)
  {
  }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart2, &buffer, RX_BUFFER_SIZE);

  if(buffer[0] == '\r' && buffer[RX_BUFFER_SIZE - 1] == '\n'){
    processCommand(buffer);
  } else{
    HAL_UART_Transmit_IT(&huart2, "Invalid command\r\n", 17);
  }

}




void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}


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



void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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

}
#endif /* USE_FULL_ASSERT */
