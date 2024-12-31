/*
 * MPU6050.h
 *
 *  Created on: Jun 4, 2023
 *      Author: Normalny
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

//#define MPU6050_ADDRESS 0x68

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


typedef struct{
	float x;
	float y;
	float z;
	float ox;
	float oy;
	float oz;
}Complementary_Filter;


struct MPU6050_Acc_Struct{
	float ax;
	float ay;
	float az;
	float ax_ang;
	float ay_ang;
	float az_ang;
	float Acc_Scale;
	float acc_x_cal;
	float acc_y_cal;
	float acc_z_cal;
};
struct MPU6050_Gyr_Struct{
	float gx;
	float gy;
	float gz;
	float Gyr_Scale;
	float gyr_x_cal;
	float gyr_y_cal;
	float gyr_z_cal;
};

typedef struct MPU6050_Struct{
	struct MPU6050_Acc_Struct Acc;
	struct MPU6050_Gyr_Struct Gyr;
	uint8_t I2C_Data[14];
	uint8_t MPU6050_IRQ;
	uint8_t Status;
}MPU6050_Struct;

enum MPU6050_Status{
	I2C_In_Use = 0x01,
	I2C_Ready = 0x02,
};


#define MPU6050_ADDRESS 0xD0

//#define A0_0 HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_RESET)
//#define A0_1 HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_SET)
//#define BUZ_0 HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET)
//#define BUZ_1 HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET)


#define MPU6050_RA_SELF_TEST_X      0x0D
#define MPU6050_RA_SELF_TEST_Y      0x0E
#define MPU6050_RA_SELF_TEST_Z      0x0F
#define MPU6050_RA_SELF_TEST_A      0x10
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
// Not in documentation
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
// Not in documentation end
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
// Not in documentation
#define MPU6050_RA_DMP_INT_STATUS   0x39
// Not in documentation end
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
// Not in documentation
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
// Not in documentation end
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
// Not in documentation
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
// Not in documentation end
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_PWR1_DEVICE_RESET_BIT   7



uint8_t MPU6050_INIT(I2C_HandleTypeDef*hi2c);
//void MPU6050_RESET(void);
void MPU6050_CONFIG_DLPF(uint8_t DLPF_CFG/*konfiguracja czenstotliwosci wysyania informacji*/);
void MPU6050_CONFIG_GYRO(uint8_t GYRO_CFG/*konfiguracja gyroskopy*/);
void MPU6050_CONFIG_ACCEL(uint8_t ACCEL_CFG/*konfiguracja akcelorometru*/);
void MPU6050_CONFIG_FIFO(void);
void MPU6050_CONFIG_INT(void);
void MPU6050_CONFIG_SAMPLE_RATE(void);
void MPU6050_CONFIG_INTERRUPT_ENABLE(void);
void MPU6050_CONFIG_USER_CONTROL(void);
void MPU6050_CALIBRATION(void);

void MPU6050_GET_ACCANDGYR_CALANDSCL(float *ax, float*ay, float*az, float*gx, float*gy, float*gz,
		float accelx_cal,float accely_cal,float accelz_cal,float gyrox_cal,float gyroy_cal,float gyroz_cal, float Gyr_Scale, float Acc_Scale);
void Complementary_getFilter(Complementary_Filter *Complementary_Filter_st,float ax_ang, float ay_ang, float az_ang, float gx_ang,
		float gy_ang, float gz_ang);
void MPU6050_VerVel_Cal(float VerAcc, float *VerVel);

void MPU6050_GET_ACCANDGYR_CALANDSCL_IT(void);
void MPU6050_GET_CALANDSCL_IT(void);

void MPU6050_GET_ACCANDGYR_FILTRED(Complementary_Filter *Complementary_Filter_st, float megz_ang);
void MPU6050_GET_GYRO_TO_ANGLE(float gx, float gy, float gz, float *gx_ang, float *gy_ang, float *gz_ang);
void MPU6050_GET_ACCEL_TO_ANGLE(void);
void MPU6050_RESET(void);
void W_Filter(Complementary_Filter *Complementary_Filter_st);
void MPU6050_Enable_I2C_Bypass(void);


uint8_t MPU6050_GET_ACCEL_XH(void);
uint8_t MPU6050_GET_ACCEL_XL(void);
uint8_t MPU6050_GET_ACCEL_YH(void);
uint8_t MPU6050_GET_ACCEL_YL(void);
uint8_t MPU6050_GET_ACCEL_ZH(void);
uint8_t MPU6050_GET_ACCEL_ZL(void);

uint8_t MPU6050_GET_GYRO_XH(void);
uint8_t MPU6050_GET_GYRO_XL(void);
uint8_t MPU6050_GET_GYRO_YH(void);
uint8_t MPU6050_GET_GYRO_YL(void);
uint8_t MPU6050_GET_GYRO_ZH(void);
uint8_t MPU6050_GET_GYRO_ZL(void);

int16_t MPU6050_GET_ACCEL_FULLVALUE_X(void);
int16_t MPU6050_GET_ACCEL_FULLVALUE_Y(void);
int16_t MPU6050_GET_ACCEL_FULLVALUE_Z(void);
int16_t MPU6050_GET_GYRO_FULLVALUE_X(void);
int16_t MPU6050_GET_GYRO_FULLVALUE_Y(void);
int16_t MPU6050_GET_GYRO_FULLVALUE_Z(void);

uint8_t MPU6050_READ_CONFIG_DLPF(void);
uint8_t MPU6050_READ_CONFIG_GYRO(void);
uint8_t MPU6050_READ_CONFIG_ACCEL(void);
uint8_t MPU6050_READ_CONFIG_FIFO(void);
uint8_t MPU6050_READ_CONFIG_INT(void);
uint8_t MPU6050_READ_CONFIG_SAMPLE_RATE(void);
uint8_t MPU6050_READ_CONFIG_INTERRUPT_ENABLE(void);
uint8_t MPU6050_READ_CONFIG_USER_CONTROL(void);
int8_t MPU6050_READ_ID(void);

void MPU6050_VerAcc_Cal(void);

void Complementary_getFilter(Complementary_Filter *Complementary_Filter_st,float ax, float ay, float magz_ang, float gx, float gy, float gz);

#endif /* INC_MPU6050_H_ */
