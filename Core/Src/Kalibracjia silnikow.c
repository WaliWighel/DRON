#include "Kalibracjia silnikow.h"

//void kalibruj_silnik(void){
//
//	for(uint8_t j = 0; j < 3; j++){
//
//		uint16_t i = 610;
//
//		while(i < 891){
//
//			ESC_SETALL(i);
//
//			if(i%2 == 0){
//				LED_G_1;
//				LED_Y_0;
//			}
//
//			if(i%2 == 1){
//				LED_G_0;
//				LED_Y_1;
//			}
//
//			i = i + 5;
//
//			HAL_Delay(10000);
//
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)analog, 1);
//			HAL_ADC_Stop_DMA(&hadc1);
//			Get_batteryvalue();
//			  if(MYDRON.dron_status.Battery == DRON_BATTERY_RUN_OUT)
//			  {
//				  LED_R_1;
//			  }
//			  if(MYDRON.dron_status.Battery == DRON_BATTERY_CRIT_VAL){
//					  ESC_POWER_1;
//					  LED_R_1;
//					  while(1){
//
//					  }
//			  }
//
//		}
//
//		LED_G_0;
//		LED_Y_0;
//		LED_R_1;
//		while(1){
//
//		}
//	}
//
//	LED_G_0;
//	LED_Y_0;
//	LED_R_1;
//	while(1){
//
//	}
//
//}
