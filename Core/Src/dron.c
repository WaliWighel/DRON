#include "main.h"

extern struct Dron MYDRON;
extern float wanted_pitch;
extern float wanted_rool;
extern float wanted_yaw;
extern int16_t wanted_thrust;
extern float old_error_pitch;
extern float old_error_rool;
extern float old_error_yaw;
extern float now_pitch;
extern float now_rool;
extern float now_yaw;
extern uint32_t analogmess;
extern float looptime;
extern float gx, gy, gz;
extern double thrust_values;
extern int32_t error_sum_pitch;
extern int32_t error_sum_rool;
extern int32_t error_sum_yaw;
extern struct Stack Old_Data_stack;
extern float PID_FAC_Pitch[5];
extern float PID_FAC_Rool[5];
extern float PID_FAC_Yaw[5];
extern int32_t error_sum_angular_rate_pitch;
extern int32_t error_sum_angular_rate_rool;
extern int32_t error_sum_angular_rate_yaw;
extern float old_error_angular_rate_pitch;// ruznica
extern float old_error_angular_rate_rool;
extern float old_error_angular_rate_yaw;
extern float PID_FAC_Angular_Rate_Pitch[5];
extern float PID_FAC_Angular_Rate_Rool[5];
extern float PID_FAC_Angular_Rate_Yaw[5];
extern float pid_pitch;
extern float pid_yaw;
extern float pid_rool;
extern int16_t wanted_pitch_rx;


/*
 * Thrust_filter
 *
 * niepozwala zwiększac ani znimiejszac ciągu jezeli error wychylenia jest zbyt duzy
 *
 */
void Thrust_filter(double factor){
	double error_pitch, error_rool, error_sum;
	double thrust_add = 0;
	int16_t thrust_error;
	double thrust_function;
	uint8_t negflag = 0;
	error_pitch = (float)(wanted_pitch - now_pitch);
	error_rool 	= (float)(wanted_rool - now_rool);

	error_pitch = WartoscBezwgledna(error_pitch);
	error_rool 	= WartoscBezwgledna(error_rool);

	error_sum = pow(error_pitch + error_rool + 1, 5);
	if(wanted_thrust > 10500){
		wanted_thrust = 10500;
	}

	thrust_error = wanted_thrust - MYDRON.THRUST;
	if(thrust_error < 0){
		thrust_error = WartoscBezwgledna(thrust_error);
		negflag = 1;
	}


	if((thrust_error) != 0){
		thrust_function = 2000*(sqrt(sqrt((double)(thrust_error))));

		thrust_add = ((double)(factor * thrust_increse_factor * thrust_function/error_sum));
		if(negflag == 1){
			thrust_add = thrust_add*(-1);
		}


		if(thrust_add < 1 && thrust_add > -1){
			thrust_values = thrust_values + thrust_add;
		}
		else{
			MYDRON.THRUST = MYDRON.THRUST + thrust_add;
		}

		if(thrust_values >= 1 || thrust_values <= -1){
			MYDRON.THRUST = MYDRON.THRUST + thrust_values;
			thrust_values = 0;
		}
		if(MYDRON.THRUST > 10000){//ograniczenie THRUST
			MYDRON.THRUST = 10000;
		}
	}
}
/*
 * Thrust_filter
 *
 * niepozwala zwiększac jezeli część D jest zbyt durza;
 * dopuszczaly 1 o/s
 *
 *	d_fac = 2000 -> d = 4
 *
 *
 */
double Thrust_filter_2(float *PID_FAC_rool, float *PID_FAC_pitch, float *PID_FAC_yaw){
	double D_Factor_sum;
	double factor = 1;// zakres od 0 do 1;

	D_Factor_sum = WartoscBezwgledna(PID_FAC_rool[4])/* + WartoscBezwgledna(PID_FAC_pitch[4]) + WartoscBezwgledna(PID_FAC_yaw[4])*/;//todo odkomentowac

	factor = (10 / ((D_Factor_sum+1)));
	if(factor > 1){
		factor = 1;
	}

	return factor;
}

float Wobble_Detect(void){
	float wobble_strenght = 1;// zakes od 1 do 1.5
	float angle_pitch_error_sum = 0;
	float angle_rool_error_sum = 0;
	float D_part_pitch = 0;
	float D_part_rool = 0;


	for(int i = 3; i < 4000; i+=4){
		angle_pitch_error_sum += Old_Data_stack.olddata[i];
	}
	for(int i = 4; i < 4000; i+=4){
		angle_rool_error_sum += Old_Data_stack.olddata[i];
	}
	angle_pitch_error_sum = angle_pitch_error_sum/1000;
	angle_rool_error_sum = angle_rool_error_sum/1000;

	for(int i = 1; i < 4000; i+=4){
		D_part_pitch += Old_Data_stack.olddata[i];
	}
	for(int i = 2; i < 4000; i+=4){
		D_part_rool += Old_Data_stack.olddata[i];
	}

	if(angle_pitch_error_sum < 1 && angle_pitch_error_sum > -1){
		if(D_part_pitch > 100000){
			wobble_strenght = 1 + (D_part_pitch/100000);
			MYDRON.dron_status.wobble = WOBBLE_PITCH;

//			if(angle_rool_error_sum < 1 && angle_rool_error_sum > -1){ todo odkomentowac
//				if(D_part_rool > 100000){
//					wobble_strenght = wobble_strenght + (D_part_rool/100000);
//					MYDRON.dron_status.wobble = WOBBLE_PITCHandROOL;
//				}
//			}
		}
	}

//	if(angle_rool_error_sum < 1 && angle_rool_error_sum > -1){ todo odkomentowac
//		if(D_part_rool > 100000){
//			wobble_strenght = 1 + (D_part_rool/100000);
//			MYDRON.dron_status.wobble = WOBBLE_ROOL;
//		}
//	}

	if(wobble_strenght > 2){
		wobble_strenght = 2;
	}
	if(wobble_strenght < 1){
		wobble_strenght = 1;
	}

	return wobble_strenght;
}

void Wobble_handler(void){
	Thrust_filter(1);
	if(MYDRON.dron_status.wobble == WOBBLE_PITCH){
//		PID_FAC_Pitch[0] = PID_FAC_Pitch[0] - 0.01;
//		PID_FAC_Pitch[2] = PID_FAC_Pitch[2] + 1;
//		if(PID_FAC_Pitch[0] < 1){
//			PID_FAC_Pitch[0] = 1;
//		}
	}

	if(MYDRON.dron_status.wobble == WOBBLE_ROOL){
//		PID_FAC_Rool[0] = PID_FAC_Rool[0] - 0.01;
//		PID_FAC_Rool[2] = PID_FAC_Rool[2] + 1;
//		if(PID_FAC_Rool[0] < 1){
//			PID_FAC_Rool[0] = 1;
//		}
	}

	if(MYDRON.dron_status.wobble == WOBBLE_PITCHandROOL){
//		PID_FAC_Pitch[0] = PID_FAC_Pitch[0] - 0.01;
//		PID_FAC_Pitch[2] = PID_FAC_Pitch[2] + 1;
//		if(PID_FAC_Pitch[0] < 1){
//			PID_FAC_Pitch[0] = 1;
//		}
//
//		PID_FAC_Rool[0] = PID_FAC_Rool[0] - 0.01;
//		PID_FAC_Rool[2] = PID_FAC_Rool[2] + 1;
//		if(PID_FAC_Rool[0] < 1){
//			PID_FAC_Rool[0] = 1;
//		}
	}
}


/*
*	zmienia wartosc P w zaleznosci od thrust
 */
float APV(float P_factor, float factor, uint8_t mov){
	float new_factor = P_factor;
	if(factor < 1){
		factor = 1;
	}
	switch(mov){
		case 1:

			new_factor = P_factor - MYDRON.THRUST/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;

		case 2:

			new_factor = P_factor - MYDRON.THRUST/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;
	}
	return new_factor;
}

/*
*	zmienia wartosc D w zaleznosci od thrust
 */
float ADV(float D_factor, float factor, uint8_t mov){
	float new_factor = D_factor;
	if(factor < 1){
		factor = 1;
	}
	switch(mov){
		case 1:

			new_factor = D_factor - MYDRON.THRUST/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;

		case 2:

			new_factor = D_factor - MYDRON.THRUST/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;
	}
	return new_factor;
}

void acceleration_stabilizer(float *g_ax, float *PID_FAC){

	*g_ax = PID_FAC[0]*(wanted_pitch - *g_ax);

	*g_ax = *g_ax + PID_FAC[1]*error_sum_pitch*looptime;

	*g_ax = *g_ax + PID_FAC[2]*((wanted_pitch - *g_ax) - old_error_pitch);

}

void PID_cal(float *PID_var, float *PID_FAC, uint8_t pry){//pitch = 1, rool = 2, yaw = 3
	switch(pry){
		case 1://pitch

			*PID_var = PID_FAC[0]*(wanted_pitch - now_pitch);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_pitch*looptime;

			PID_FAC[3] = PID_FAC[2]*((wanted_pitch - now_pitch) - old_error_pitch);//policzenie częsci D

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];

				if(*PID_var > 400){//PID_var jest w o/s, jezeli bendzei chcailo sie obracac szybciej niz.. przekroczy zakres pomiarowy akcelerometru
					*PID_var = 400;
				}
				if(*PID_var < -400){
					*PID_var = -400;
				}
			break;

		case 2://rool

			*PID_var = PID_FAC[0]*(wanted_rool - now_rool);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_rool*looptime;

			PID_FAC[3] =  PID_FAC[2]*((wanted_rool - now_rool) - old_error_rool);

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];

			if(*PID_var > 400){
				*PID_var = 400;
			}
			if(*PID_var < -400){
				*PID_var = -400;
			}
			break;

		case 3:

			*PID_var = PID_FAC[0]*(wanted_yaw - now_yaw);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_yaw*looptime;

			PID_FAC[3] = PID_FAC[2]*((wanted_yaw - now_yaw) - old_error_yaw);

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];

			if(*PID_var > 400){
				*PID_var = 400;
			}
			if(*PID_var < -400){
				*PID_var = -400;
			}
			break;

		case 4:// angular rates pitch

			*PID_var = PID_FAC[0]*(pid_pitch - gx);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_pitch*looptime;


			PID_FAC[3] = PID_FAC[2]*((pid_pitch - gx) - old_error_angular_rate_pitch);//policzenie częsci D

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];

			break;

		case 5:// angular rates rool

			*PID_var = PID_FAC[0]*(pid_rool - gy);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_rool*looptime;

			PID_FAC[3] =  PID_FAC[2]*((pid_rool - gy) - old_error_angular_rate_rool);

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];
			break;

		case 6:// angular rates yaw

			*PID_var = PID_FAC[0]*(pid_yaw - gz);

			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_yaw*looptime;

			PID_FAC[3] = PID_FAC[2]*((pid_yaw - gz) - old_error_angular_rate_yaw);

			//FDP
			//PID_FAC[4] = (PID_FAC[4] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[5] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
			PID_FAC[4] = PID_FAC[3];//old d_fac

			*PID_var = *PID_var + PID_FAC[3];
			break;
		default:
			break;
	}
}

void Get_batteryvalue(void){
	/*
	 * Kiedy 9,5V -> DRON_BATTERY_RUN_OUT 9,6V kryyczne napięcie bateri
	 * dzielnik to 3,7
	 * Max napiencie to 11,3V to po dzielniku 3,05v czyli 3786
	 * 8V po dzielniku to 2,16V czyli 2681
	 * nie 8V
	 * 10V musi byc czyli 2,7027V
	 *	2,7027V to 3353
	 *	3400
	*/

	MYDRON.batterysize = (analogmess - 3353)/7.42;
	if(MYDRON.batterysize >= 100){
		MYDRON.batterysize = 100;
	}

	if(MYDRON.batterysize < 25){
		MYDRON.dron_status.Battery = DRON_BATTERY_RUN_OUT;
	}
	if(MYDRON.batterysize <= 10){
		MYDRON.dron_status.Battery = DRON_BATTERY_CRIT_VAL;
	}
	if(MYDRON.batterysize >= 25){
		MYDRON.dron_status.Battery = DRON_BATTERY_OK;
	}
}

void IS_DRON_ON_GROUND(void){//todo
	int x = 0;

	if(MYDRON.dronheight > 1){
		x++;
	}

	if(1){
		x++;
	}
}

void PID_Self_Regulation(uint8_t freedom, float *PID_FAC){
	uint8_t best = 0;
	float overshoot[2];//[0] - teraz [1] poprzedni
	overshoot[0] = 0;
	overshoot[1] = 0;
	 switch(freedom){
	 	 case 1://pitch angular
	 		 for(int i = 0; i < 5; i++){
	 			 PID_FAC[i] = 0;
	 		 }
	 		 while(best != 1){
	 			 PID_FAC[0] = 1;//ustawienie wspulczynika P

	 			 if(overshoot[0] < overshoot[1]){

	 			 }
	 		 }
	 		 break;
	 	 case 2://pitch
	 		 break;
	 	 case 3://rool angular
	 		 break;
	 	 case 4://rool
	 		 break;
	 	 case 5://yaw angular
	 		 break;
	 	 case 6://yaw
	 		 break;
	 }
}

/*float Calculate_altitude(float ax, float ay, float az, float last_altitude){
	float G_Force = 0;
	float altitude = 0;

	G_Force = (ax + ay + az) - 255;

	altitude =+ last_altitude;

	return altitude;
}*/

/*void Position_Holding(void){
	float G_Force = 0;
	float X_acc = 0, Y_acc = 0, Z_acc = 0;

	//G_Force = (ax + ay + az) - 255;//suma przyspieszeń - przyspieszenie ziemskie(255)\

	X_acc = ax;
	Y_acc = ay;
	Z_acc = G_Force - X_acc - Y_acc;
}*/
