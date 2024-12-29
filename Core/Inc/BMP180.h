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


//extern int32_t temperature, pressure;
//extern float temp, pres, ampritude;

struct BMP180_Callibration_Regs_Struct{
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	uint32_t B4;
	uint32_t B7;
	uint32_t UP;
	int32_t UT;
	int32_t X1;
	int32_t X2;
	int32_t B5;
	int32_t B6;
	int32_t B3;
	int32_t X3;
};

struct BMP180_Raw_Data_Struct{
	int32_t temperature;
	int32_t pressure;
};

struct BMP180_Struct{
	struct BMP180_Callibration_Regs_Struct Callibration_Regs;
	struct BMP180_Raw_Data_Struct Raw_Data;
	float temp;
	float pres;
	float ampritude;
	float startpres;
	uint8_t Data_Press_IT[3];
	uint8_t Data_Temp_IT[2];
	uint8_t BMP180_IRQ;
	uint8_t I2C_Tx_IRQ;
	uint8_t Timer;
};

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
