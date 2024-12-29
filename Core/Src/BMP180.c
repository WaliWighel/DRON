/*
 * BMP180.c
 *
 *  Created on: Jul 10, 2023
 *      Author: Normalny
 */
#include "main.h"

static I2C_HandleTypeDef *hi2c_BMP180;
extern struct BMP180_Struct BMP180;
//extern uint8_t BMP180_Press_IT[3], BMP180_Temp_IT[2];
//extern uint8_t BMP180_IRQ;

uint8_t BMP180_read_ID(void){// comunication = 0x55
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, id_register, 1, &data, 1, 100);
	return data;
}

uint8_t BMP180_init(I2C_HandleTypeDef*hi2c){
	hi2c_BMP180 = hi2c;
	uint8_t data;
	BMP180_read_calliberation_data();
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, id_register, 1, &data, 1, 100);
	data = data | 0x40;
	HAL_I2C_Mem_Write(hi2c_BMP180, BMP180_ADDRES, 0xF4, 1, &data, 1, 1);//set oversampling to 2

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
	BMP180.Callibration_Regs.UT = ((data[0]<<8) | data[1]);
	return BMP180.Callibration_Regs.UT;
}

uint32_t BMP180_READ_pres(void){
	uint8_t data[3];
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, data, 3, 1000);
	return (((data[0]<<16)|(data[1]<<8)|data[2]) >> 5);
}

void BMP180_READ_temp_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, (uint8_t *)BMP180.Data_Temp_IT, 2);
	BMP180.BMP180_IRQ = 1;
}

void BMP180_READ_pres_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_BMP180, BMP180_ADDRES, out_msb, 1, (uint8_t *)BMP180.Data_Press_IT, 3);
	BMP180.BMP180_IRQ = 2;
}

uint16_t BMP180_GET_temp_IT(void){
	BMP180.Callibration_Regs.UT = ((BMP180.Data_Temp_IT[0]<<8) | BMP180.Data_Temp_IT[1]);
	return BMP180.Callibration_Regs.UT;
}

uint32_t BMP180_GET_pres_IT(void){
	return (((BMP180.Data_Press_IT[0]<<16)|(BMP180.Data_Press_IT[1]<<8)|BMP180.Data_Press_IT[2]) >> 5);
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
	BMP180.I2C_Tx_IRQ = 2;
}

void BMP180_start_measurment_temp_IT(void){
	uint8_t data = 0x2E;
	HAL_I2C_Mem_Write_IT(hi2c_BMP180, BMP180_ADDRES, ctrl_meas, 1, &data, 1);
	BMP180.I2C_Tx_IRQ = 1;
}

void BMP180_read_calliberation_data(void){

	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(hi2c_BMP180, BMP180_ADDRES, Callib_Start, 1, Callib_Data, 22, 100);

	BMP180.Callibration_Regs.AC1 = (int16_t)((Callib_Data[0] << 8) | Callib_Data[1]);
	BMP180.Callibration_Regs.AC2 = (int16_t)((Callib_Data[2] << 8) | Callib_Data[3]);
	BMP180.Callibration_Regs.AC3 = (int16_t)((Callib_Data[4] << 8) | Callib_Data[5]);
	BMP180.Callibration_Regs.AC4 = (int16_t)((Callib_Data[6] << 8) | Callib_Data[7]);
	BMP180.Callibration_Regs.AC5 = (int16_t)((Callib_Data[8] << 8) | Callib_Data[9]);
	BMP180.Callibration_Regs.AC6 = (int16_t)((Callib_Data[10] << 8) | Callib_Data[11]);
	BMP180.Callibration_Regs.B1 = (int16_t)((Callib_Data[12] << 8) | Callib_Data[13]);
	BMP180.Callibration_Regs.B2 = (int16_t)((Callib_Data[14] << 8) | Callib_Data[15]);
	BMP180.Callibration_Regs.MB = (int16_t)((Callib_Data[16] << 8) | Callib_Data[17]);
	BMP180.Callibration_Regs.MC = (int16_t)((Callib_Data[18] << 8) | Callib_Data[19]);
	BMP180.Callibration_Regs.MD = (int16_t)((Callib_Data[20] << 8) | Callib_Data[21]);
}

float BMP180_GET_temp(uint16_t temperature){
	float temp;
	BMP180.Callibration_Regs.X1 = ((temperature - BMP180.Callibration_Regs.AC6) * BMP180.Callibration_Regs.AC5)/32768;
	BMP180.Callibration_Regs.X2 = (BMP180.Callibration_Regs.MC * 2048)/(BMP180.Callibration_Regs.X1 + BMP180.Callibration_Regs.MD);
	BMP180.Callibration_Regs.B5 = BMP180.Callibration_Regs.X1 + BMP180.Callibration_Regs.X2;
	temp = (BMP180.Callibration_Regs.B5 + 8)/16;
	return temp/10.0;
}

float BMP180_GET_pres(uint16_t pressure){
	float pres;
	BMP180.Callibration_Regs.UP = BMP180_READ_pres();
	BMP180.Callibration_Regs.X1 = (((BMP180.Callibration_Regs.UT-BMP180.Callibration_Regs.AC6) * BMP180.Callibration_Regs.AC5)/32768);//
	BMP180.Callibration_Regs.X2 = ((BMP180.Callibration_Regs.MC*(2048)) / (BMP180.Callibration_Regs.X1+BMP180.Callibration_Regs.MD));
	BMP180.Callibration_Regs.B5 = BMP180.Callibration_Regs.X1+BMP180.Callibration_Regs.X2;
	BMP180.Callibration_Regs.B6 = BMP180.Callibration_Regs.B5-4000;
	BMP180.Callibration_Regs.X1 = (BMP180.Callibration_Regs.B2 * (BMP180.Callibration_Regs.B6*BMP180.Callibration_Regs.B6/(4096)))/(2048);
	BMP180.Callibration_Regs.X2 = BMP180.Callibration_Regs.AC2*BMP180.Callibration_Regs.B6/(2048);
	BMP180.Callibration_Regs.X3 = BMP180.Callibration_Regs.X1+BMP180.Callibration_Regs.X2;
	BMP180.Callibration_Regs.B3 = (((BMP180.Callibration_Regs.AC1*4+BMP180.Callibration_Regs.X3)<<3)+2)/4;
	BMP180.Callibration_Regs.X1 = BMP180.Callibration_Regs.AC3*BMP180.Callibration_Regs.B6/8192;
	BMP180.Callibration_Regs.X2 = (BMP180.Callibration_Regs.B1 * (BMP180.Callibration_Regs.B6*BMP180.Callibration_Regs.B6/(4096)))/(65536);
	BMP180.Callibration_Regs.X3 = ((BMP180.Callibration_Regs.X1+BMP180.Callibration_Regs.X2)+2)/4;
	BMP180.Callibration_Regs.B4 = BMP180.Callibration_Regs.AC4* (uint32_t)(BMP180.Callibration_Regs.X3+32768)/(32768);
	BMP180.Callibration_Regs.B7 = ((uint32_t)BMP180.Callibration_Regs.UP-BMP180.Callibration_Regs.B3)*(50000>>3);
	if (BMP180.Callibration_Regs.B7 < 0x80000000){
		pres = (BMP180.Callibration_Regs.B7*2)/BMP180.Callibration_Regs.B4;
	}
	else{
		pres = (BMP180.Callibration_Regs.B7/BMP180.Callibration_Regs.B4)*2;
	}
	BMP180.Callibration_Regs.X1 = (pres/(256))*(pres/(256));
	BMP180.Callibration_Regs.X1 = (BMP180.Callibration_Regs.X1*3038)/(65536);
	BMP180.Callibration_Regs.X2 = (-7357*pres)/(65536);
	pres = pres + (BMP180.Callibration_Regs.X1+BMP180.Callibration_Regs.X2+3791)/(16);
	return pres;
}

void BMP180_CALIBRATION(float *firstpres){
	float temperature, pressure, pres;
	//for(int i = 0; i < 30; i++){
	BMP180_start_measurment_temp();
	HAL_Delay(10); // 9
	temperature = BMP180_READ_temp();
	BMP180.temp = BMP180_GET_temp(temperature);
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
	height = (BMP180.ampritude/factor);
	return height;
}
