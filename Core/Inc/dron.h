/*
 * dron.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Normalny
 */

#ifndef INC_DRON_H_
#define INC_DRON_H_

#define thrust_increse_factor 	0.0003 // w procentach od 0-100% np. 10% = 0.1
#define thrust_stability_factor 1


struct Dronstatus{
	uint16_t Connection;
	uint16_t Battery;
	uint16_t Code;
	uint16_t engines;
	uint16_t position;
	uint16_t wobble;
};

struct Dron{
	uint16_t batterysize;
	uint16_t THRUST;
	int16_t	PITCH;
	int16_t ROOL;
	int16_t YAW;
	struct Dronstatus dron_status;
	int16_t dronheight;
	uint8_t THRUST_flag;
};

enum DRON_StateTypeDef{
	DRON_CONNECTED = 0x01,
	DRON_DISCONNECTED = 0x02,
	DRON_CONNECTION_ERROR = 0x03,

    DRON_CODE_HARDFAULT_ERROR = 0x04,
	DRON_CODE_OK = 0x05,
	DRON_CODE_STUCK = 0x06,

	DRON_BATTERY_RUN_OUT = 0x07,
	DRON_BATTERY_OK = 0x08,
	DRON_BATTERY_CRIT_VAL = 0x09,

	DRON_ENGINE_ERROR = 0x00,
	DRON_ENGINE_OK = 0x0A,
	DRON_ENGINE_BIG_CURRENT = 0x0B,

	DRON_POSITION_OK = 0x0C,
	DRON_POSITION_WRONG = 0x0D,

	THRUST_MAX = 0x10,

	NO_WOBBLE = 0x0E,
	WOBBLE_PITCH = 0x0F,
	WOBBLE_ROOL = 0x11,
	WOBBLE_PITCHandROOL = 0x012
};

struct Stack{   // sapisanie danych z ostatniej sekundy
	float olddata[4010];
	uint16_t start_pointer;
	uint16_t end_pointer;
};

void PID_cal(float *PID_var, float *PID_FAC, uint8_t pry);
void IS_DRON_ON_GROUND(void);
void Get_batteryvalue(void);
void Thrust_filter(double factor);
void acceleration_stabilizer(float *g_ax, float *PID_FAC);
float APV(float P_factor, float factor, uint8_t mov);
float ADV(float D_factor, float factor, uint8_t mov);
double  Thrust_filter_2(float *PID_FAC_rool, float *PID_FAC_pitch, float *PID_FAC_yaw);
float Wobble_Detect(void);
void Wobble_handler(void);

#endif /* INC_DRON_H_ */
