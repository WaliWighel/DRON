/*
 * USART komendy.h
 *
 *  Created on: Aug 15, 2024
 *      Author: Normalny
 */

#ifndef INC_USART_KOMENDY_H_
#define INC_USART_KOMENDY_H_

extern uint8_t commandready;
extern uint8_t words[];
extern char command[];
extern uint8_t UASRT_PID_VAL[];

extern uint16_t FDP_D_Gain_AR;
extern uint16_t FDP_D_Gain;

void interpretcommand(void);
void executecommand(char command[], uint8_t value1[]);


#endif /* INC_USART_KOMENDY_H_ */
