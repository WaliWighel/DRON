/*
 * HMC5883L.h
 *
 *  Created on: Sep 2, 2024
 *      Author: Normalny
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_


#define HMC5883L_I2C_Address 0x1E //!!!!!!!!!!!!!!!!!!!!!!! shift to the left (HMC5883L_I2C_Address<<1)

#define HMC5883L_I2C_Read_Address 0x3D
//#define HMC5883L_I2C_Write_Address 0x3C

#define HMC5883L_Configuration_Register_A 0x00
#define HMC5883L_Configuration_Register_B 0x01
#define HMC5883L_Mode_Register 0x02

#define HMC5883L_Data_Output_X_MSB_Register 0x03
#define HMC5883L_Data_Output_X_LSB_Register 0x04
#define HMC5883L_Data_Output_Z_MSB_Register 0x05
#define HMC5883L_Data_Output_Z_LSB_Register 0x06
#define HMC5883L_Data_Output_Y_MSB_Register 0x07
#define HMC5883L_Data_Output_Y_LSB_Register 0x08

#define HMC5883L_Status_Register 0x09

#define HMC5883L_Identifaction_Register_A 0x0A
#define HMC5883L_Identifaction_Register_B 0x0B
#define HMC5883L_Identifaction_Register_C 0x0C




uint8_t HMC5883L_Init(I2C_HandleTypeDef*hi2c);

int16_t HMC5883L_Get_Z_Start(void);
void HMC5883L_Get_Z_Start_IT(void);
void HMC5883L_Get_Z_End_IT(void);
int16_t HMC5883L_Calibration(void);


#endif /* INC_HMC5883L_H_ */
