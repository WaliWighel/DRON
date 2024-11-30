/*
 * HMC5883L.c
 *
 *  Created on: Sep 2, 2024
 *      Author: Normalny
 */


#include "main.h"


static I2C_HandleTypeDef *hi2c_HMC5883L;

extern uint8_t HMC5883L_Data_IT[6];
extern uint8_t HMC583L_IRQ;

uint8_t HMC5883L_Init(I2C_HandleTypeDef*hi2c){

	hi2c_HMC5883L = hi2c;

	uint8_t data = 0;

	/*
	 * bit7 = 1
	 * bit6 = 1		( 8 samples
	 * bit5 = 1		)
	 * bit4 = 1 	{
	 * bit3 = 0			30 HzOutput rate
	 * bit2 = 0		}
	 * bit1 = 0
	 * bit0 = 0
	 */
	data = 0xF0;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Configuration_Register_A, 1, &data, 1, 1);
	/*
	 *
	 * bit7 = 0	{
	 * bit6 = 0		+-1.3 Ga (1090 LSB/Gauss)
	 * bit5 = 1 }
	 * bit4 = 0
	 * bit3 = 0
	 * bit2 = 0
	 * bit1 = 0
	 * bit0 = 0
	 */
	data = 0x32;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Configuration_Register_B, 1, &data, 1, 1);
	/*
	 *
	 * bit7 = 1
	 * bit6 = 0
	 * bit5 = 0
	 * bit4 = 0
	 * bit3 = 0
	 * bit2 = 0
	 * bit1 = 0	{ Continuous-Measurement Mode
	 * bit0 = 0 }
	 */
	data = 0x80;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Mode_Register, 1, &data, 1, 1);


	uint8_t status = 0;

	HAL_I2C_Mem_Read(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Identifaction_Register_A, 1, &status, 1, 1);

	if(status != 0x48){
		status = 0;
	}
	else{
		status = 1;
	}
	HAL_Delay(10);

	return status;
}


int16_t HMC5883L_Get_Z_Start(void){
	int16_t fulldata = 182;
	uint8_t data[6];

	HAL_I2C_Mem_Read(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Data_Output_Z_MSB_Register, 1, data, 6, 1);

	fulldata = ((int16_t)data[4]<<8) | data[5];

	return fulldata;
}

void HMC5883L_Get_Z_Start_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Data_Output_Z_MSB_Register, 1, (uint8_t *)HMC5883L_Data_IT, 6);
	HMC583L_IRQ = 1;
}

int16_t HMC5883L_Get_Z_End_IT(void){
	int16_t fulldata = 0;

	fulldata = ((int16_t)HMC5883L_Data_IT[4]<<8) | HMC5883L_Data_IT[5];

	return fulldata;
}


int16_t HMC5883L_Calibration(void){
	int64_t mes_data = 0;
	for(int i = 0; i < 10; i++){
		mes_data += HMC5883L_Get_Z_Start();
		HAL_Delay(100); //Output = 30Hz
	}

	return (mes_data/10);
}
