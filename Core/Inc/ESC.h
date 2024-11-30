#ifndef ESC_H_
#define ESC_H_

#include "main.h"

#define max_speed 19500//960todo
#define min_speed 10500//12540//615





#define ESC_POWER_1 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_SET)//1 = unucitve
#define ESC_POWER_0 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_RESET)//0 = active

void ESC_1_SPEED(uint16_t a);
void ESC_2_SPEED(uint16_t a);
void ESC_3_SPEED(uint16_t a);
void ESC_4_SPEED(uint16_t a);

void ESC_SETALL(uint16_t a);

void ESC_INT(TIM_HandleTypeDef *htim);

#endif
