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
//extern float p_pitchfactor;
//extern float i_pitchfactor;
//extern float d_pitchfactor;
//extern float p_angular_rate_pitchfactor;
//extern float i_angular_rate_pitchfactor;
//extern float d_angular_rate_pitchfactor;
//extern int32_t error_sum_pitch;
//extern int32_t error_sum_angular_rate_pitch;

//extern float p_roolfactor;
//extern float i_roolfactor;
//extern float d_roolfactor;
//extern float p_angular_rate_roolfactor;
//extern float i_angular_rate_roolfactor;
//extern float d_angular_rate_roolfactor;
//extern int32_t error_sum_rool;
//extern int32_t error_sum_angular_rate_rool;


//extern float p_yawfactor;
//extern float i_yawfactor;
//extern float d_yawfactor;
//extern float p_angular_rate_yawfactor;
//extern float i_angular_rate_yawfactor;
//extern float d_angular_rate_yawfactor;
//extern int32_t error_sum_yaw;
//extern int32_t error_sum_angular_rate_yaw;


extern uint16_t FDP_D_Gain_AR;
extern uint16_t FDP_D_Gain;

void interpretcommand(void);
void executecommand(char command[], uint8_t value1[]);


#endif /* INC_USART_KOMENDY_H_ */
