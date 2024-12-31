/*
 * MPU6050.c
 *
 *  Created on: Jun 4, 2023
 *      Author: Normalny
 */
#include "main.h"
#include "math.h"
#include "MPU6050.h"
#include "i2c.h"


static I2C_HandleTypeDef *hi2c_mpu6050;

extern float looptime;
extern MPU6050_Struct MPU6050;



uint8_t MPU6050_INIT(I2C_HandleTypeDef*hi2c){
	hi2c_mpu6050 = hi2c;
	MPU6050_RESET();
	MPU6050_CONFIG_SAMPLE_RATE();
	MPU6050_CONFIG_DLPF(0x05);// //05
	/*
	 *  0 - 250 o/s
	 *  0x08 - 500 o/s
	 */
	MPU6050_CONFIG_GYRO(0x08);//+-500 o/s
	MPU6050_CONFIG_ACCEL(0x08); // +-4g

	MPU6050_Enable_I2C_Bypass();
	MPU6050_CONFIG_USER_CONTROL();

	HAL_Delay(10);

	uint8_t status;
	status = MPU6050_READ_ID();

	if(status != 104){
		status = 0;//error
	}
	else{
		status = 1;//ok
	}


//	int x = MPU6050_GET_ACCEL_FULLVALUE_X();
//	x = MPU6050_READ_CONFIG_SAMPLE_RATE();
//	x = MPU6050_READ_CONFIG_DLPF();//1kHz;
//	x = MPU6050_READ_CONFIG_GYRO();//+-500 o/s;
//	x = MPU6050_READ_CONFIG_ACCEL(); // +-8g;;
	return status;
}


void MPU6050_RESET(void){
	uint8_t data = 0x00;//0x80
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, 1);
}

void MPU6050_CONFIG_DLPF(uint8_t DLPF_CFG){
	uint8_t data = DLPF_CFG;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &data, 1, 1);
}

void MPU6050_CONFIG_GYRO(uint8_t GYRO_CFG/*konfiguracja gyroskopy*/){
	uint8_t data = GYRO_CFG;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &data, 1, 1);
}

void MPU6050_CONFIG_ACCEL(uint8_t ACCEL_CFG/*konfiguracja akcelorometru*/){
	uint8_t data = ACCEL_CFG;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &data, 1, 1);
}

void MPU6050_CONFIG_FIFO(void){
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 1, &data, 1, 1);
}

void MPU6050_CONFIG_INT(void){
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &data, 1, 1);
}

void MPU6050_CONFIG_SAMPLE_RATE(void){
	uint8_t data = 0x00; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)                 Gyroscope Output Rate = 1kHz
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1, &data, 1, 1);
}

void MPU6050_CONFIG_INTERRUPT_ENABLE(void){
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &data, 1, 1);
}

void MPU6050_CONFIG_USER_CONTROL(void){
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &data, 1, 1);
}

//void MPU6050_RESET(uint8_t Reset){
//	uint8_t tmp;
//	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, 100);
//	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
//	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
//	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, 100);
//}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU6050_CALIBRATION(void){
	float accelx = 0, accely = 0, accelz = 0, gyrox = 0, gyroy = 0, gyroz = 0;
	for(int i = 0; i < 1000; i++){ //5 sec
		  accelx = accelx + MPU6050_GET_ACCEL_FULLVALUE_X()/MPU6050.Acc.Acc_Scale;
		  accely = accely + MPU6050_GET_ACCEL_FULLVALUE_Y()/MPU6050.Acc.Acc_Scale;
		  accelz = accelz + MPU6050_GET_ACCEL_FULLVALUE_Z()/MPU6050.Acc.Acc_Scale;

		  gyrox = gyrox + MPU6050_GET_GYRO_FULLVALUE_X()/MPU6050.Gyr.Gyr_Scale;
		  gyroy = gyroy + MPU6050_GET_GYRO_FULLVALUE_Y()/MPU6050.Gyr.Gyr_Scale;
		  gyroz = gyroz + MPU6050_GET_GYRO_FULLVALUE_Z()/MPU6050.Gyr.Gyr_Scale;
		  HAL_Delay(1);
	}
	MPU6050.Acc.acc_x_cal = accelx/1000;
	MPU6050.Acc.acc_y_cal = accely/1000;
	MPU6050.Acc.acc_z_cal = accelz/1000;

	MPU6050.Acc.acc_z_cal = 1 - MPU6050.Acc.acc_z_cal;

	MPU6050.Gyr.gyr_x_cal = gyrox/1000;
	MPU6050.Gyr.gyr_y_cal = gyroy/1000;
	MPU6050.Gyr.gyr_z_cal = gyroz/1000;
}

void MPU6050_GET_ACCEL_TO_ANGLE(void){
	float ang1 = sqrt((MPU6050.Acc.ax*MPU6050.Acc.ax)+(MPU6050.Acc.az*MPU6050.Acc.az));
	float ang2 = sqrt((MPU6050.Acc.ay*MPU6050.Acc.ay)+(MPU6050.Acc.az*MPU6050.Acc.az));

	MPU6050.Acc.ay_ang = ((-1*(atan(MPU6050.Acc.ax/ang2)))*180)/M_PI;
	MPU6050.Acc.ax_ang = ((atan(MPU6050.Acc.ay/ang1))*180)/M_PI;
//	axan = -1*(atan(ax/ang2));
//	ayan= atan(ay/ang1);
//
//	*ay_ang = (axan*180)/M_PI;
//	*ax_ang = (ayan*180)/M_PI;
}

void MPU6050_GET_GYRO_TO_ANGLE(float gx, float gy, float gz, float *gx_ang, float *gy_ang, float *gz_ang){
	*gx_ang = (gx/1000) + *gx_ang;
	*gy_ang = (gy/1000) + *gy_ang;
	*gz_ang = (gz/1000) + *gz_ang;
}

void MPU6050_GET_ACCANDGYR_CALANDSCL(float *ax, float*ay, float*az, float*gx, float*gy, float*gz, float accelx_cal,float accely_cal,
		float accelz_cal,float gyrox_cal,float gyroy_cal,float gyroz_cal, float Gyr_Scale, float Acc_Scale){

	uint8_t pdata[14];
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, pdata, 14, 1); // szybesz o ~1ms

	//HAL_I2C_Mem_Read_IT(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, pdata, 14);

	*ax = (((int16_t)(pdata[0]<<8) | pdata[1])/Acc_Scale) - accelx_cal;
	*ay = (((int16_t)(pdata[2]<<8) | pdata[3])/Acc_Scale) - accely_cal;
	*az = (((int16_t)(pdata[4]<<8) | pdata[5])/Acc_Scale) + accelz_cal;

	*gx = (((int16_t)(pdata[8]<<8) | pdata[9])/Gyr_Scale) - gyrox_cal;
	*gy = (((int16_t)(pdata[10]<<8) | pdata[11])/Gyr_Scale) - gyroy_cal;
	*gz = (((int16_t)(pdata[12]<<8) | pdata[13])/Gyr_Scale) - gyroz_cal;

//	  *ax = (MPU6050_GET_ACCEL_FULLVALUE_X()/Acc_Scale) - accelx_cal;
//	  *ay = (MPU6050_GET_ACCEL_FULLVALUE_Y()/Acc_Scale) - accely_cal;
//	  *az = (MPU6050_GET_ACCEL_FULLVALUE_Z()/Acc_Scale) + accelz_cal;
//
//	  *gx = (MPU6050_GET_GYRO_FULLVALUE_X()/Gyr_Scale) - gyrox_cal;
//	  *gy = (MPU6050_GET_GYRO_FULLVALUE_Y()/Gyr_Scale) - gyroy_cal;
//	  *gz = (MPU6050_GET_GYRO_FULLVALUE_Z()/Gyr_Scale) - gyroz_cal;
}

void MPU6050_GET_ACCANDGYR_CALANDSCL_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, MPU6050.I2C_Data, 14);
	MPU6050.MPU6050_IRQ = 1;
}

void MPU6050_GET_CALANDSCL_IT(void){
	MPU6050.Acc.ax = (((int16_t)(MPU6050.I2C_Data[0]<<8) | MPU6050.I2C_Data[1])/MPU6050.Acc.Acc_Scale) - MPU6050.Acc.acc_x_cal;
	MPU6050.Acc.ay = (((int16_t)(MPU6050.I2C_Data[2]<<8) | MPU6050.I2C_Data[3])/MPU6050.Acc.Acc_Scale) - MPU6050.Acc.acc_y_cal;
	MPU6050.Acc.az = (((int16_t)(MPU6050.I2C_Data[4]<<8) | MPU6050.I2C_Data[5])/MPU6050.Acc.Acc_Scale) + MPU6050.Acc.acc_z_cal;

	MPU6050.Gyr.gx = (((int16_t)(MPU6050.I2C_Data[8]<<8) | MPU6050.I2C_Data[9])/MPU6050.Gyr.Gyr_Scale) - MPU6050.Gyr.gyr_x_cal;
	MPU6050.Gyr.gy = (((int16_t)(MPU6050.I2C_Data[10]<<8) | MPU6050.I2C_Data[11])/MPU6050.Gyr.Gyr_Scale) - MPU6050.Gyr.gyr_y_cal;
	MPU6050.Gyr.gz = (((int16_t)(MPU6050.I2C_Data[12]<<8) | MPU6050.I2C_Data[13])/MPU6050.Gyr.Gyr_Scale) - MPU6050.Gyr.gyr_z_cal;
}

void MPU6050_GET_ACCANDGYR_FILTRED(Complementary_Filter *Complementary_Filter_st, float megz_ang){
	Complementary_getFilter(Complementary_Filter_st, MPU6050.Acc.ax_ang, MPU6050.Acc.ay_ang, megz_ang, MPU6050.Gyr.gx, MPU6050.Gyr.gy, MPU6050.Gyr.gz);
	//W_Filter(Complementary_Filter_st);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t MPU6050_GET_ACCEL_XH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_ACCEL_XL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_ACCEL_YH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_ACCEL_YL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_ACCEL_ZH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_ACCEL_ZL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_GET_GYRO_XH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_GYRO_XL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_GYRO_YH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_GYRO_YL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_GYRO_ZH(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, &data, 1, 1);
	return data;
}
uint8_t MPU6050_GET_GYRO_ZL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, 1, &data, 1, 1);
	return data;
}
int16_t MPU6050_GET_ACCEL_FULLVALUE_X(void){
	int16_t data;
	uint8_t pdata[2];

	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, pdata, 2, 1);

	data = (pdata[0]<<8)|pdata[1];
	return data;
}

int16_t MPU6050_GET_ACCEL_FULLVALUE_Y(void){
	int16_t data;
	data = (MPU6050_GET_ACCEL_YH()<<8)|MPU6050_GET_ACCEL_YL();
	return data;
}
int16_t MPU6050_GET_ACCEL_FULLVALUE_Z(void){
	int16_t data;
	data = (MPU6050_GET_ACCEL_ZH()<<8)|MPU6050_GET_ACCEL_ZL();
	return data;
}

int16_t MPU6050_GET_GYRO_FULLVALUE_X(void){
	int16_t data;
	data = (MPU6050_GET_GYRO_XH()<<8)|MPU6050_GET_GYRO_XL();
	return data;
}
int16_t MPU6050_GET_GYRO_FULLVALUE_Y(void){
	int16_t data;
	data = (MPU6050_GET_GYRO_YH()<<8)|MPU6050_GET_GYRO_YL();
	return data;
}
int16_t MPU6050_GET_GYRO_FULLVALUE_Z(void){
	int16_t data;
	data = (MPU6050_GET_GYRO_ZH()<<8)|MPU6050_GET_GYRO_ZL();
	return data;
}

int8_t MPU6050_READ_ID(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, 1, &data, 1, 1);
	return data;
}



uint8_t MPU6050_READ_CONFIG_DLPF(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_GYRO(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_ACCEL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_FIFO(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_INT(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_SAMPLE_RATE(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_INTERRUPT_ENABLE(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &data, 1, 1);
	return data;
}

uint8_t MPU6050_READ_CONFIG_USER_CONTROL(void){
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &data, 1, 1);
	return data;
}


void MPU6050_Enable_I2C_Bypass(void){
	uint8_t data = 0x02;
	HAL_I2C_Mem_Write(hi2c_mpu6050, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, 1);
}

void MPU6050_VerAcc_Cal(void){//todo
//	float VerAcc;
//	(az - 1) * 9.81 * 1000;// up acc when x and y = 0; -1 becouse of 1g output is in mm/s
//	VerAcc = ((az - 1) * 9.81 * 100) * ax * ay;
}
void MPU6050_VerVel_Cal(float VerAcc, float *VerVel){
	*VerVel += (VerAcc*looptime);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Complementary_getFilter(Complementary_Filter *Complementary_Filter_st,float ax_ang, float ay_ang, float magz_ang, float gx_ang, float gy_ang, float gz_ang){

		Complementary_Filter_st->x = (0.02*ax_ang)+(0.98*((gx_ang*looptime)+Complementary_Filter_st->ox));//
		Complementary_Filter_st->y = (0.02*ay_ang)+(0.98*((gy_ang*looptime)+Complementary_Filter_st->oy));
		Complementary_Filter_st->z = (((gz_ang*looptime)+Complementary_Filter_st->oz));//todo add magnetomiter?

		Complementary_Filter_st->ox = Complementary_Filter_st->x;
		Complementary_Filter_st->oy = Complementary_Filter_st->y;
		Complementary_Filter_st->oz = Complementary_Filter_st->z;


//	for(int i = 1; i < 15; i++){
//		OldXs[i] = OldXs[i-1];
//	}
//	OldXs[0] = Complementary_Filter_st->ox;
//
//	for(int i = 1; i < 15; i++){
//		OldYs[i] = OldYs[i-1];
//	}
//	OldYs[0] = Complementary_Filter_st->oy;
//
//	for(int i = 1; i < 15; i++){
//		OldZs[i] = OldZs[i-1];
//	}
//	OldZs[0] = Complementary_Filter_st->oz;
}


//void W_Filter(Complementary_Filter *Complementary_Filter_st){
//
//	float sum = 0;
//
//	for(int i = 0; i < 16; i++){
//		sum = sum + OldXs[i];
//	}
//
//	xval = sum/16;
//
//	if(xval - Complementary_Filter_st->x >= 1 || xval - Complementary_Filter_st->x <= -1){
//		if((OldXs[1] + OldXs[0] + Complementary_Filter_st->x) > (OldXs[4] + OldXs[3] + OldXs[2]+ 3) || (OldXs[1] + OldXs[0] + Complementary_Filter_st->x) < (OldXs[4] + OldXs[3]+OldXs[2] - 3)){
//			Complementary_Filter_st->x = Complementary_Filter_st->x;
//		}
//	}
//	else{
//		Complementary_Filter_st->x = xval;
//	}
//
//
//
//	sum = 0;
//
//	for(int i = 0; i < 16; i++){
//		sum = sum + OldYs[i];
//	}
//
//	yval = sum/16;
//
//	if(yval - Complementary_Filter_st->y >= 1 || yval - Complementary_Filter_st->y <= -1){
//		if((OldYs[1] + OldYs[0] + Complementary_Filter_st->y) > (OldYs[4] + OldYs[3] + OldYs[2]) || (OldYs[1] + OldYs[0] + Complementary_Filter_st->y) < (OldYs[4] + OldYs[3]+OldYs[2])){
//			Complementary_Filter_st->y = Complementary_Filter_st->y;
//		}
//	}
//	else{
//		Complementary_Filter_st->y = yval;
//	}
//
//
//
//	sum = 0;
//
//	for(int i = 0; i < 16; i++){
//		sum = sum + OldZs[i];
//	}
//
//	zval = sum/16;
//
//	if(zval - Complementary_Filter_st->z >= 1 || zval - Complementary_Filter_st->z <= -1){
//		if((OldZs[1] + OldZs[0] + Complementary_Filter_st->z) > (OldZs[4] + OldZs[3] + OldZs[2]) || (OldZs[1] + OldZs[0] + Complementary_Filter_st->z) < (OldZs[4] + OldZs[3]+OldZs[2])){
//			Complementary_Filter_st->z = Complementary_Filter_st->z;
//		}
//	}
//	else{
//		Complementary_Filter_st->z = zval;
//	}
//}
