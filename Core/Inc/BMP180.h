/*
 * BMP180.h
 *
 *  Created on: Jul 10, 2023
 *      Author: Normalny
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#define BMP180_ADDRES 0xEE
//#define BMP180_ADDRES_read 0xEF

extern uint16_t AC4, AC5, AC6;
extern int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
extern uint32_t B4, B7, UP;
extern int32_t temperature, pressure, UT, X1, X2, B5, B6, B3, X3;
extern float temp, pres, ampritude;

// registers
#define out_xlsb 0xF8
#define out_lsb 0xF7
#define out_msb 0xF6
#define ctrl_meas 0xF4
#define soft_reset 0xE0
#define id_register 0xD0

uint8_t BMP180_init(I2C_HandleTypeDef*hi2c);
uint8_t BMP180_read_ID(void);
uint16_t BMP180_READ_temp(void);
uint32_t BMP180_READ_pres(void);

void BMP180_READ_temp_IT(void);
void BMP180_READ_pres_IT(void);

uint16_t BMP180_GET_temp_IT(void);
uint32_t BMP180_GET_pres_IT(void);


void BMP180_start_measurment_temp(void);
void BMP180_start_measurment_pres(void);
void BMP180_start_measurment_pres_IT(void);
void BMP180_start_measurment_temp_IT(void);
void BMP180_read_calliberation_data(void);
float BMP180_GET_temp(uint16_t temperature);
float BMP180_GET_pres(uint16_t pressure);
void BMP180_CALIBRATION(float *firstpres);
float BMP180_GET_height(void);

#endif /* INC_BMP180_H_ */
