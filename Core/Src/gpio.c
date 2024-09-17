/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"

#include "led.h"

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PC13_GPIO_Port, PC13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nCS_Pin|ETHreset_Pin|Buzzer_Pin|WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Red_Pin|Yellow_Pin|Green_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = ETHint_Pin|Can_Pin|Pair_Pin|Clear_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Call_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Call_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = nCS_Pin|ETHreset_Pin|Buzzer_Pin|Red_Pin
                          |Yellow_Pin|Green_Pin|WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = Pendant2_Pin|Pendant1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
uint16_t ID = 0x03;
uint8_t csend[] = {0x55, 0xA5, 0xCD, 0x5A, 0x23, 0x99, 0xBD, 0x00};
volatile uint8_t state_gpio[SIZE_ENUM];
volatile uint8_t cnt_state_pressed = NONE;

void init_state_buf()
{
  for (uint8_t idx=0;idx<SIZE_ENUM;idx++)
  {
    state_gpio[idx] = 0;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case ETHint_Pin:
    break;

  case Can_Pin:
    if (cnt_state_pressed == NONE) break;
#ifdef debug
    printf("Press cancel\r\n");
#endif
    csend[0] = 0x00;
    if (state_gpio[NURSE_PRESENCE]){
      csend[1] = NURSE_PRESENCE;
      state_gpio[NURSE_PRESENCE] = 0;
      sendCanODL(&ID, csend);
      cnt_state_pressed--;
      HAL_GPIO_WritePin(GPIOB, Green_Pin, GPIO_PIN_SET);
    }
    if (state_gpio[EMERGENCY]){
      csend[1] = EMERGENCY;
      state_gpio[EMERGENCY] = 0;
      sendCanODL(&ID, csend);
      cnt_state_pressed--;
      HAL_GPIO_WritePin(GPIOB, Red_Pin, GPIO_PIN_SET);
    }
    if (state_gpio[PENDANT]){
      csend[1] = PENDANT;
      state_gpio[PENDANT] = 0;
      sendCanODL(&ID, csend);
      cnt_state_pressed--;
      HAL_GPIO_WritePin(GPIOB, Yellow_Pin, GPIO_PIN_SET);
    }
    if (state_gpio[DOCTOR_PRESENCE]){
      csend[1] = DOCTOR_PRESENCE;
      state_gpio[DOCTOR_PRESENCE] = 0;
      sendCanODL(&ID, csend);
      cnt_state_pressed--;
    }
    setBuzzTime(100);
    break;

  case Pendant1_Pin:
    if (state_gpio[PENDANT] || HAL_GPIO_ReadPin(Pendant1_GPIO_Port, Pendant1_Pin)) break;
#ifdef debug
    printf("Press PENDANT1\r\n");
#endif
    csend[0] = PENDANT;
    state_gpio[PENDANT] = 1;
  	Set_LED(3, 254, 140, 0);
  	Set_Brightness(15);
    sendCanODL(&ID, csend);
    cnt_state_pressed++;
    setBuzzTime(140);
    break;
  case Pendant2_Pin:
    if (state_gpio[PENDANT] || HAL_GPIO_ReadPin(Pendant2_GPIO_Port, Pendant2_Pin)) break;
#ifdef debug
    printf("Press PENDANT2\r\n");
#endif
    break;

  case Clear_Pin:
    if (state_gpio[DOCTOR_PRESENCE]) break;
#ifdef debug
    printf("Press clear\r\n");
#endif
    // csend[0] = DOCTOR_PRESENCE;
    // state_gpio[DOCTOR_PRESENCE] = 1;
    // sendCanODL(&ID, csend);
    // cnt_state_pressed++;
    break;

  case Pair_Pin:
#ifdef debug
    printf("Press pair\r\n");
#endif
    // csend[0] = 0x04;
    // sendCanODL(&ID, csend);
    break;

  default:
    break;
  }
}

#define bounceDelay 20            // Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 3          // Number of times the button has to be detected as pressed before the press is considered to be valid
#define maxButtonPress 30         // Number of times the button has to be detected as pressed before the press is considered to be valid
volatile uint32_t previousMillis; // Timers to time out bounce duration for each button
volatile uint8_t pressCount;
volatile uint16_t buzz = 0;
volatile uint32_t currentMillis;
void call_debounce()
{
  currentMillis = millis();
  if (HAL_GPIO_ReadPin(Call_GPIO_Port, Call_Pin) == GPIO_PIN_SET) // release botton
  {
    if ((minButtonPress < pressCount) && (pressCount < maxButtonPress))
    {
      if (state_gpio[NURSE_PRESENCE]) return;
#ifdef debug
      printf("Press green\r\n");
#endif
      csend[0] = NURSE_PRESENCE;
      state_gpio[NURSE_PRESENCE] = 1;

  	Set_LED(4, 34, 139, 34);
  	Set_Brightness(15);
  	WS2812_Send();
      sendCanODL(&ID, csend);

      HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin,
                        GPIO_PIN_RESET);
      cnt_state_pressed++;
    }
    previousMillis = currentMillis; // Set previousMillis to millis to reset timeout
    pressCount = 0;                 // Set the number of times the button has been detected as pressed to 0
  }
  else
  {
    // BEEP for green state
    if (currentMillis - previousMillis > bounceDelay)
    {
      previousMillis = currentMillis; // Set previousMillis to millis to reset timeout
      ++pressCount;
      if (pressCount == minButtonPress)
      {
        buzz = 10;
      }
      // red state condition
      if (pressCount == maxButtonPress)
      {
        if (state_gpio[EMERGENCY]) return;
#ifdef debug
        printf("Press red\r\n");
#endif
        csend[0] = EMERGENCY;
        state_gpio[EMERGENCY] = 1;
      	Set_LED(5, 255, 0, 0);
      	Set_Brightness(15);
      	WS2812_Send();
        sendCanODL(&ID, csend);

        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin,
                          GPIO_PIN_RESET);
        cnt_state_pressed++;
        buzz = 40;
      }
    }
  }
}

volatile uint32_t _buzzMillis = 0;
void setBuzzTime(uint16_t onTime)
{
  buzz = (uint16_t)(onTime / bounceDelay);
}

volatile uint32_t previousBuzzMillis = 0;
void buzzCheck()
{
  if (buzz)
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  if (currentMillis - previousBuzzMillis > bounceDelay)
  {
    previousBuzzMillis = currentMillis; // Set previousMillis to millis to reset timeout
    if (buzz)
    {
      buzz--;
    }
  }
}
lamp_state_t getButtonState(void)
{
    if (state_gpio[NURSE_PRESENCE]) return NURSE_PRESENCE;
    if (state_gpio[EMERGENCY]) return EMERGENCY;
    if (state_gpio[PENDANT]) return PENDANT;
    if (state_gpio[PULLCORD]) return PULLCORD;
    return NONE;
}

/* USER CODE END 2 */
