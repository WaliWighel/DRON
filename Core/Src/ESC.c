#include "main.h"
#include "tim.h"

#include "ESC.h"

static TIM_HandleTypeDef *htim_ESC; //450Hz



void ESC_1_SPEED(uint16_t a)
{
	  if(a < min_speed)
	  {
		  a = min_speed;
	  }
	  if(a >  max_speed)
	  {
		  a =  max_speed;
	  }
	  __HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_1, a);
}

void ESC_2_SPEED(uint16_t a)
{
	  if(a < min_speed - Engine2_balance)
	  {
		  a = min_speed - Engine2_balance;
	  }
	  if(a >  max_speed - Engine2_balance)
	  {
		  a =  max_speed - Engine2_balance;
	  }

	  __HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_2, a - Engine2_balance);
}

void ESC_3_SPEED(uint16_t a)
{
	  if(a < min_speed - Engine3_balance)
	  {
		  a = min_speed - Engine3_balance;
	  }
	  if(a >  max_speed - Engine3_balance)
	  {
		  a =  max_speed - Engine3_balance;
	  }

	  __HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_3, a - Engine3_balance);
}

void ESC_4_SPEED(uint16_t a)
{
	  if(a < min_speed - Engine4_balance)
	  {
		  a = min_speed - Engine4_balance;
	  }
	  if(a > max_speed - Engine4_balance)
	  {
		  a = max_speed - Engine4_balance;
	  }

	  __HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_4, a - Engine4_balance);
}

void ESC_SETALL(uint16_t a)
{
	ESC_1_SPEED(a);
	ESC_2_SPEED(a);
	ESC_3_SPEED(a);
	ESC_4_SPEED(a);
}


/*  ESC_INT
 *
 * najpierw wlonczamy PWM a dopiero potem zasilanie do silników, w przeciwnym razie ESC mogą włączyć się w trybie programowania.
 *
 */
void ESC_INT(TIM_HandleTypeDef *htim)
{
	htim_ESC = htim;


	HAL_TIM_PWM_Start(htim_ESC, TIM_CHANNEL_1);//450Hz
	HAL_TIM_PWM_Start(htim_ESC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim_ESC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim_ESC, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_4, 10000);
	__HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_3, 10000);
	__HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_2, 10000);
	__HAL_TIM_SET_COMPARE(htim_ESC, TIM_CHANNEL_1, 10000);
	ESC_POWER_0;
	HAL_Delay(3000);
}


/*
 * ESC_Calibration
 *
 * Wypełnienie PWM na MAX, po pewnym czasie ustawic PWM na najnizszy zakres czyli 50%
 * ESC_Calibration nalezy zrobic zamiast ESC_INT
 *
 */
void ESC_Calibration(void){

}
