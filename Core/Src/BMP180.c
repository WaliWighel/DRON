/*
 * BMP180.c
 *
 *  Created on: Jul 10, 2023
 *      Author: Normalny
 */
#include "main.h"

static I2C_HandleTypeDef *hi2c_BMP180;
extern uint8_t BMP180_Press_IT[3], BMP180_Temp_IT[2];
extern uint8_t BMP180_IRQ;

uint8_t BMP180_read_ID(void){// comunication = 0x55
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, id_register, 1, &data, 1, 100);
	return data;
}

uint8_t BMP180_init(I2C_HandleTypeDef*hi2c){
	hi2c_BMP180 = hi2c;
	BMP180_read_calliberation_data();

	uint8_t status = 0;
	status = BMP180_read_ID();

	if(status != 0x55){
		status = 0;
	}
	else{
		status = 1;
	}

	return status;
}

uint16_t BMP180_READ_temp(void){
	uint8_t data[2];
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, data, 2, 100);
	UT = ((data[0]<<8) | data[1]);
	return UT;
}

uint32_t BMP180_READ_pres(void){
	uint8_t data[3];
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, data, 3, 1000);
	return (((data[0]<<16)|(data[1]<<8)|data[2]) >> 5);
}

void BMP180_READ_temp_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, (uint8_t *)BMP180_Temp_IT, 2);
	BMP180_IRQ = 1;
}

void BMP180_READ_pres_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, (uint8_t *)BMP180_Press_IT, 3);
	BMP180_IRQ = 2;
}

uint16_t BMP180_GET_temp_IT(void){
	UT = ((BMP180_Temp_IT[0]<<8) | BMP180_Temp_IT[1]);
	return UT;
}

uint32_t BMP180_GET_pres_IT(void){
	return (((BMP180_Press_IT[0]<<16)|(BMP180_Press_IT[1]<<8)|BMP180_Press_IT[2]) >> 5);
}

//void BMP180_measurment(uint16_t *temp, uint16_t *pres){
//	uint16_t temperature, pressure;
//	BMP180_start_measurment_temp();
//	//HAL_Delay(5);
//	temperature = BMP180_READ_temp();
//	//HAL_Delay(5);
//	BMP180_start_measurment_pres();
//	//HAL_Delay(26);
//	pressure = BMP180_READ_pres();
//}

void BMP180_start_measurment_temp(void){
	uint8_t data = 0x2E;
	HAL_I2C_Mem_Write(hi2c_BMP180, BMP180_ADDRES, ctrl_meas, 1, &data, 1, 100);
}

void BMP180_start_measurment_pres(void){
	uint8_t data = 0xF4;
	HAL_I2C_Mem_Write(hi2c_BMP180, BMP180_ADDRES, ctrl_meas, 1, &data, 1, 100);
}

void BMP180_start_measurment_pres_IT(void){
	uint8_t data = 0xF4;
	HAL_I2C_Mem_Write_IT(hi2c_BMP180, BMP180_ADDRES, ctrl_meas, 1, &data, 1);
}

void BMP180_start_measurment_temp_IT(void){
	uint8_t data = 0x2E;
	HAL_I2C_Mem_Write_IT(hi2c_BMP180, BMP180_ADDRES, ctrl_meas, 1, &data, 1);
}

void BMP180_read_calliberation_data(void){

	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, Callib_Start, 1, Callib_Data, 22, 100);

	AC1 = (int16_t)((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = (int16_t)((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = (int16_t)((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = (int16_t)((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = (int16_t)((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = (int16_t)((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = (int16_t)((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = (int16_t)((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = (int16_t)((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = (int16_t)((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = (int16_t)((Callib_Data[20] << 8) | Callib_Data[21]);
}

float BMP180_GET_temp(uint16_t temperature){

	X1 = ((temperature - AC6) * AC5)/32768;
	X2 = (MC * 2048)/(X1 + MD);
	B5 = X1 + X2;
	temp = (B5 + 8)/16;
	return temp/10.0;
}

float BMP180_GET_pres(uint16_t pressure){

	UP = BMP180_READ_pres();
	X1 = (((UT-AC6) * AC5)/32768);//
	X2 = ((MC*(2048)) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(4096)))/(2048);
	X2 = AC2*B6/(2048);
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<3)+2)/4;
	X1 = AC3*B6/8192;
	X2 = (B1 * (B6*B6/(4096)))/(65536);
	X3 = ((X1+X2)+2)/4;
	B4 = AC4* (uint32_t)(X3+32768)/(32768);
	B7 = ((uint32_t)UP-B3)*(50000>>3);
	if (B7 < 0x80000000){
		pres = (B7*2)/B4;
	}
	else{
		pres = (B7/B4)*2;
	}
	X1 = (pres/(256))*(pres/(256));
	X1 = (X1*3038)/(65536);
	X2 = (-7357*pres)/(65536);
	pres = pres + (X1+X2+3791)/(16);
	return pres;
}

void BMP180_CALIBRATION(float *firstpres){

	//for(int i = 0; i < 30; i++){
	BMP180_start_measurment_temp();
	HAL_Delay(10); // 9
	temperature = BMP180_READ_temp();
	temp = BMP180_GET_temp(temperature);
	BMP180_start_measurment_pres();
	HAL_Delay(30); // 30
	pressure = BMP180_READ_pres();
	pres = BMP180_GET_pres(pressure);
	//}
	*firstpres = pres;
}

float BMP180_GET_height(void){
	float height = 0, factor; // metry
	factor = 11.3; // na 1m cisnienie spada o 11,3 pa
	height = (ampritude/factor);
	return height;
}
