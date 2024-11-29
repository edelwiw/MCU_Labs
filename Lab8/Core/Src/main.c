/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
#define NUM_INDICATORS 8

// 7 seg display digits
//                GFEDCBA
#define DIGIT_N 0b0000000 // 0
#define DIGIT_0 0b0111111 // 63
#define DIGIT_1 0b0000110 // 6
#define DIGIT_2 0b1011011 // 91
#define DIGIT_3 0b1001111 // 79
#define DIGIT_4 0b1100110 // 102
#define DIGIT_5 0b1101101 // 109
#define DIGIT_6 0b1111101 // 125
#define DIGIT_7 0b0000111 // 7
#define DIGIT_8 0b1111111 // 127
#define DIGIT_9 0b1101111 // 111
#define DOT 0b10000000 // 128

#define DS3231_ADDR 0x68
enum DS3231_REG{
  SECONDS = 0x00,
  MINUTES = 0x01,
  HOURS = 0x02,
  DAY = 0x03,
  DATE = 0x04,
  MONTH = 0x05,
  YEAR = 0x06,
};

struct time_t {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint8_t year;
};


struct time_t get_time;
struct time_t decoded_time;

struct time_t set_time; 


uint8_t digits[] = {DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4, DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t timer; 

// uint32_t display_number = 0;
uint8_t display_buffer[NUM_INDICATORS] = {0}; 
uint8_t current_digit = 0;

uint8_t send_buffer[2] = {0};

char read_buffer[NUM_INDICATORS + 3];
uint8_t read_index = 0;
uint8_t display_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10); 


  HAL_UART_Receive_IT(&huart2, (uint8_t *)&read_buffer[read_index++], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
    // char buffer[100];
    // sprintf(buffer, "date: %02d.%02d.%02d, time: %02d:%02d:%02d\n", decoded_time.date, decoded_time.month, decoded_time.year, decoded_time.hours, decoded_time.minutes, decoded_time.seconds);
    // HAL_UART_Transmit_IT(&huart2, (uint8_t *)buffer, strlen(buffer)); 

    // HAL_Delay(1000);

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
      display_flag = 0;
    }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET){
      display_flag = 1;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 79;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|SPI2_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SPI2_SS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// timer 10 interrupt handler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10)
  { 
    timer++;
    if(timer % 100 == 0){
      read_time();
    }
   
    // update display
    HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_RESET);
    uint8_t data = display_buffer[current_digit];
    uint8_t digit = ~(1 << current_digit);
    send_buffer[0] = data;
    send_buffer[1] = digit;
    HAL_SPI_Transmit_DMA(&hspi2, send_buffer, 2);
    current_digit = (current_digit + 1) % NUM_INDICATORS;
  }
}

void read_time(void){
    HAL_I2C_Mem_Read_DMA(&hi2c1, DS3231_ADDR << 1, SECONDS, 1, &get_time, sizeof(struct time_t));
}

// i2c 1 interrupt handler
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  if (hi2c->Instance == I2C1){
    decode_time(&decoded_time, &get_time);
    display(&decoded_time);
  }
}

// spi 2 interrupt handler
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_SET);
}


// uart 2 interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if(read_buffer[read_index - 1] == '\n' || read_buffer[read_index - 1] == '\r') {
      read_buffer[read_index - 1] = '\0';

    process_command(read_buffer);
    read_index = 0;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&read_buffer[read_index++], 1);
  }
}

uint8_t addr;
char buffer[100];
uint8_t arg;

void process_command(char *command){
  arg = atoi(&command[1]);
  switch (command[0]){
  case 'h':
    // set hours
    if(arg > 23 || arg < 0){
      return;
    }
    addr = HOURS;
    sprintf(buffer, "set hours: %d\n", arg);
    break;
  case 'm':
    // set minutes
    if(arg > 59 || arg < 0){
      return;
    }
    addr = MINUTES;
    sprintf(buffer, "set minutes: %d\n", arg);
    break;
  case 's':
    // set seconds
    if(arg > 59 || arg < 0){
      return;
    }
    addr = SECONDS;
    sprintf(buffer, "set seconds: %d\n", arg);
    break;
  
  case 'Y':
    // set year
    if(arg > 99 || arg < 0){
      return;
    }
    addr = YEAR;
    sprintf(buffer, "set year: %d\n", arg);
    break;
  case 'M':
    // set month
    if(arg > 12 || arg < 1){
      return;
    }
    addr = MONTH;
    sprintf(buffer, "set month: %d\n", arg);
    break;
  case 'D':
    // set date
    if(arg > 31 || arg < 1){
      return;
    }
    addr = DATE;
    sprintf(buffer, "set date: %d\n", arg);
    break;
  
  default:
    return;
  }

  arg = ((arg / 10) << 4) | (arg % 10);

  HAL_UART_Transmit_IT(&huart2, (uint8_t *)buffer, strlen(buffer));
  HAL_I2C_Mem_Write_DMA(&hi2c1, DS3231_ADDR << 1, addr, 1, &arg, 1);
}

void display(struct time_t *decoded_time){
  char dig[NUM_INDICATORS + 1]; 

  if(display_flag){
    sprintf(dig, "%02d%02d%02d", decoded_time->hours, decoded_time->minutes, decoded_time->seconds);
  } else {
    sprintf(dig, "%02d%02d%02d", decoded_time->date, decoded_time->month, decoded_time->year);
  }

  uint8_t num_digits = strlen(dig);
  for(int i = 0; i < NUM_INDICATORS; i++){
    if(i < num_digits){
      display_buffer[i] = digits[dig[i] - '0'] | (i % 2 == 1 ? (1 << 7) : 0);
    } else {
      display_buffer[i] = DIGIT_N;
    }
  }

  // char buffer[100];
  // sprintf(buffer, "display: %d %d %d %d %d %d %d %d\n", display_buffer[0], display_buffer[1], display_buffer[2], display_buffer[3], display_buffer[4], display_buffer[5], display_buffer[6], display_buffer[7]);
  // HAL_UART_Transmit_IT(&huart2, (uint8_t *)buffer, strlen(buffer));
}


void decode_time(struct time_t *decoded_time, struct time_t *get_time)
{
  decoded_time->seconds = ((get_time->seconds & 0xF0 )>> 4) * 10 + (get_time->seconds & 0x0F);
  decoded_time->minutes = ((get_time->minutes & 0xF0 )>> 4) * 10 + (get_time->minutes & 0x0F);
  decoded_time->hours = ((get_time->hours & 0xF0 )>> 4) * 10 + (get_time->hours & 0x0F);
  
  decoded_time->date = ((get_time->date & 0xF0 )>> 4) * 10 + (get_time->date & 0x0F);
  decoded_time->month = ((get_time->month & 0x70 )>> 4) * 10 + (get_time->month & 0x0F);
  decoded_time->year = ((get_time->year & 0xF0 )>> 4) * 10 + (get_time->year & 0x0F);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
