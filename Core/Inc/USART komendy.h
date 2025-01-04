/*
 * USART komendy.h
 *
 *  Created on: Aug 15, 2024
 *      Author: Normalny
 */

#ifndef INC_USART_KOMENDY_H_
#define INC_USART_KOMENDY_H_

extern uint16_t FDP_D_Gain_AR;
extern uint16_t FDP_D_Gain;

void interpretcommand(void);
void executecommand(char command[], uint8_t value1[]);

typedef struct USART_Struct{
	uint8_t UASRT_PID_VAL[15];
	uint8_t commandready;
	uint8_t words[10];
	uint8_t Received;
	uint8_t command_ch_num;
	char command[1];
}USART_Struct;


#endif /* INC_USART_KOMENDY_H_ */
