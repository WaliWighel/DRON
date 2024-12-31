/*
 * dron.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Normalny
 */

#ifndef INC_DRON_H_
#define INC_DRON_H_

#define thrust_increse_factor 	0.0006 // w procentach od 0-100% np. 10% = 0.1
#define thrust_stability_factor 1


struct Dronstatus{
	uint8_t Connection;
	uint8_t Battery;
	uint8_t Code;
	uint8_t Wobble;
};

struct Dron_Pitch{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	int32_t Angle_Error_Sum;
	int32_t Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Rool{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	int32_t Angle_Error_Sum;
	int32_t Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Yaw{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	int32_t Angle_Error_Sum;
	int32_t Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Thrust{
	uint16_t Now;
	int16_t Wanted;
	int16_t Thrust_Limit;
	double Values;
	uint16_t Speed_1;
	uint16_t Speed_2;
	uint16_t Speed_3;
	uint16_t Speed_4;
	uint16_t Old_Speed_1;
	uint16_t Old_Speed_2;
	uint16_t Old_Speed_3;
	uint16_t Old_Speed_4;
	uint8_t Max_Flag;
};
struct PID_Pitch{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[5];
	float Angular_Rate_Factors[5];
	uint8_t Status;
};
struct PID_Rool{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[5];
	float Angular_Rate_Factors[5];
	uint8_t Status;
};
struct PID_Yaw{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[5];
	float Angular_Rate_Factors[5];
	uint8_t Status;
};

typedef struct Dron{
	struct Dron_Pitch Pitch;
	struct Dron_Rool Rool;
	struct Dron_Yaw Yaw;
	struct Dron_Thrust Thrust;
	struct PID_Pitch PID_Pitch;
	struct PID_Rool PID_Rool;
	struct PID_Yaw PID_Yaw;
	struct Dronstatus Status;
	uint16_t batterysize;
	int16_t dronheight;
}Dron;

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
	WOBBLE_PITCHandROOL = 0x12
};

typedef struct Stack{   // sapisanie danych z ostatniej sekundy
	float olddata[4010];
	uint16_t start_pointer;
	uint16_t end_pointer;
}Stack;

void PID_cal(float *PID_var, float *PID_FAC, uint8_t pry);
void PID_call(Dron Paramiters);
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
