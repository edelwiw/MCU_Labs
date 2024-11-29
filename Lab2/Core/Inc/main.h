/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE BEGIN Private defines */
#define Bt1_Pin GPIO_PIN_13
#define Bt1_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_4
#define Led1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_5
#define Led2_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_6
#define Led3_GPIO_Port GPIOC
#define Led4_Pin GPIO_PIN_7
#define Led4_GPIO_Port GPIOC
#define Led5_Pin GPIO_PIN_8
#define Led5_GPIO_Port GPIOC
#define Led6_Pin GPIO_PIN_9
#define Led6_GPIO_Port GPIOC
#define BtLed1_Pin GPIO_PIN_15
#define BtLed1_GPIO_Port GPIOA
#define Led7_Pin GPIO_PIN_10
#define Led7_GPIO_Port GPIOC
#define Led8_Pin GPIO_PIN_11
#define Led8_GPIO_Port GPIOC
#define BtLed2_Pin GPIO_PIN_12
#define BtLed2_GPIO_Port GPIOC
#define Bt2_Pin GPIO_PIN_2
#define Bt2_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
