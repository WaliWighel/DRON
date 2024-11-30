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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP180.h"
#include "NRF24_Defs.h"
#include "ESC.h"
#include "NRF24.h"
#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "dron.h"
#include "math.h"
#include "Kalibracjia silnikow.h"
#include "USART komendy.h"
#include "HMC5883L.h"
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
#define LED_6_Pin GPIO_PIN_3
#define LED_6_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOE
#define LED_Y_Pin GPIO_PIN_10
#define LED_Y_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define uSD_Detection_Pin GPIO_PIN_12
#define uSD_Detection_GPIO_Port GPIOB
#define uSD_LED_Pin GPIO_PIN_11
#define uSD_LED_GPIO_Port GPIOD
#define uSD_SC_Pin GPIO_PIN_15
#define uSD_SC_GPIO_Port GPIOD
#define ESC_Power_Pin GPIO_PIN_8
#define ESC_Power_GPIO_Port GPIOG
#define LED_7_Pin GPIO_PIN_8
#define LED_7_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_15
#define LED_5_GPIO_Port GPIOA
#define NRF24_IRQ_Pin GPIO_PIN_15
#define NRF24_IRQ_GPIO_Port GPIOG
#define NRF24_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define NRF24_CE_Pin GPIO_PIN_6
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_CSN_Pin GPIO_PIN_7
#define NRF24_CSN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void convert_value_to_array(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);
void convert_array_to_value(uint8_t arrayfrom[], int16_t *value , uint8_t rangebegin, uint8_t rangeend);
uint32_t potenga(int a, int b);

void valswitch(uint8_t);
float WartoscBezwgledna(float a);

void RGB_LED_Set_color(uint8_t R, uint8_t G, uint8_t B);
void RGB_LED_For_BAT(uint8_t batval);
void uSD_Card_SendData_To_Buffer(uint32_t a);
void convert_value_to_array2(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);
void convert_value_to_array3(float value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);
void Stack_Push(float data);

int16_t ROOL_MAX_VAL(void);
int16_t ROOL_MIN_VAL(void);
int16_t PITCH_MAX_VAL(void);
int16_t PITCH_MIN_VAL(void);
int16_t YAW_MAX_VAL(void);
int16_t YAW_MIN_VAL(void);
int16_t YAW_GOOD_VAL(void);
int16_t PITCH_GOOD_VAL(void);
int16_t ROOL_GOOD_VAL(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
