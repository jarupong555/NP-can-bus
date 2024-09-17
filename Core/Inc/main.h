/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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
void sendNS(const uint32_t id, uint8_t data[]);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define ETHint_Pin GPIO_PIN_0
#define ETHint_GPIO_Port GPIOA
#define ETHint_EXTI_IRQn EXTI0_IRQn
#define Call_Pin GPIO_PIN_1
#define Call_GPIO_Port GPIOA
#define Can_Pin GPIO_PIN_2
#define Can_GPIO_Port GPIOA
#define Can_EXTI_IRQn EXTI2_IRQn
#define Pair_Pin GPIO_PIN_3
#define Pair_GPIO_Port GPIOA
#define Pair_EXTI_IRQn EXTI3_IRQn
#define Clear_Pin GPIO_PIN_4
#define Clear_GPIO_Port GPIOA
#define Clear_EXTI_IRQn EXTI4_IRQn
#define nCS_Pin GPIO_PIN_0
#define nCS_GPIO_Port GPIOB
#define ETHreset_Pin GPIO_PIN_1
#define ETHreset_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOB
#define Red_Pin GPIO_PIN_10
#define Red_GPIO_Port GPIOB
#define Yellow_Pin GPIO_PIN_11
#define Yellow_GPIO_Port GPIOB
#define Green_Pin GPIO_PIN_12
#define Green_GPIO_Port GPIOB
#define Pendant2_Pin GPIO_PIN_13
#define Pendant2_GPIO_Port GPIOB
#define Pendant2_EXTI_IRQn EXTI15_10_IRQn
#define Pendant1_Pin GPIO_PIN_14
#define Pendant1_GPIO_Port GPIOB
#define Pendant1_EXTI_IRQn EXTI15_10_IRQn
#define WP_Pin GPIO_PIN_15
#define WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define debug 0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
