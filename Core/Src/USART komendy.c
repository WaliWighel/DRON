#include "main.h"

void interpretcommand(void){

	uint8_t vcount = 0, J1 = 0;
	char value1[10];


	commandready = 0;

	for(int j = 0; j < 80; j++){

		if(words[j] == ' '){
			vcount++;
		}
		if(vcount == 0){
			command[j] = words[j];
		}
		if(vcount == 1){
			value1[J1] = words[j];
			UASRT_PID_VAL[J1] = (int)value1[J1];//;printf("%d", value11[i]);
			J1++;
		}
	}
}

void executecommand(char command[], uint8_t value1[]){

	if(command[0] == 'P')
	{
		p_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
	}

	if(command[0] == 'I')
	{
		i_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
		error_sum_pitch = 0;
	}

	if(command[0] == 'D')
	{
		d_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'p')
	{
		p_angular_rate_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
	}

	if(command[0] == 'i')
	{
		i_angular_rate_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
		error_sum_angular_rate_pitch = 0;
	}

	if(command[0] == 'd')
	{
		d_angular_rate_pitchfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'F')
	{
		FDP_D_Gain_AR = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'f')
	{
		FDP_D_Gain = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
	}





//rool
	if(command[0] == 'a')
		{
			p_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
		}

		if(command[0] == 'b')
		{
			i_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
			error_sum_rool = 0;
		}

		if(command[0] == 'c')
		{
			d_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
		}
		if(command[0] == 'e')
		{
			p_angular_rate_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
		}

		if(command[0] == 'g')
		{
			i_angular_rate_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
			error_sum_angular_rate_rool = 0;
		}

		if(command[0] == 'h')
		{
			d_angular_rate_roolfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
		}








		if(command[0] == 'j')
			{
				p_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
			}

			if(command[0] == 'k')
			{
				i_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
				error_sum_yaw = 0;
			}

			if(command[0] == 'l')
			{
				d_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
			}
			if(command[0] == 'm')
			{
				p_angular_rate_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
			}

			if(command[0] == 'n')
			{
				i_angular_rate_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*100) + ((((float)UASRT_PID_VAL[2])-48)*10) + ((((float)UASRT_PID_VAL[3])-48)) + ((((float)UASRT_PID_VAL[4])-48)/10) + ((((float)UASRT_PID_VAL[5])-48)/100);
				error_sum_angular_rate_yaw = 0;
			}

			if(command[0] == 'o')
			{
				d_angular_rate_yawfactor = ((((float)UASRT_PID_VAL[1])-48)*1000) + ((((float)UASRT_PID_VAL[2])-48)*100) + ((((float)UASRT_PID_VAL[3])-48)*10) + ((((float)UASRT_PID_VAL[4])-48)) + ((((float)UASRT_PID_VAL[5])-48)/10);
			}

	if(command[0] == 'r'){
		NVIC_SystemReset();
	}

	for(int i = 0; i < 15; i++){
		UASRT_PID_VAL[i] = 0;
	}
//	for(int i = 0; i < 10; i++){
//		words[i] = 0;
//	}

}
