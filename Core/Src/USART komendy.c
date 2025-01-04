#include "main.h"

extern USART_Struct USART;

void interpretcommand(void){

	uint8_t vcount = 0, J1 = 0;
	char value1[10];


	USART.commandready = 0;

	for(int j = 0; j < 80; j++){

		if(USART.words[j] == ' '){
			vcount++;
		}
		if(vcount == 0){
			USART.command[j] = USART.words[j];
		}
		if(vcount == 1){
			value1[J1] = USART.words[j];
			USART.UASRT_PID_VAL[J1] = (int)value1[J1];//;printf("%d", value11[i]);
			J1++;
		}
	}
}

void executecommand(char command[], uint8_t value1[]){

	if(command[0] == 'P')
	{
		MYDRON.PID_Pitch.Angle_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
	}

	if(command[0] == 'I')
	{
		MYDRON.PID_Pitch.Angle_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
		MYDRON.Pitch.Angle_Error_Sum = 0;
	}

	if(command[0] == 'D')
	{
		MYDRON.PID_Pitch.Angle_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'p')
	{
		MYDRON.PID_Pitch.Angular_Rate_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
	}

	if(command[0] == 'i')
	{
		MYDRON.PID_Pitch.Angular_Rate_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
		MYDRON.Pitch.Angular_Rate_Error_Sum = 0;
	}

	if(command[0] == 'd')
	{
		MYDRON.PID_Pitch.Angular_Rate_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'F')
	{
		FDP_D_Gain_AR = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
	}
	if(command[0] == 'f')
	{
		FDP_D_Gain = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
	}





//rool
	if(command[0] == 'a')
		{
			MYDRON.PID_Rool.Angle_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
		}

		if(command[0] == 'b')
		{
			MYDRON.PID_Rool.Angle_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
			MYDRON.Pitch.Angle_Error_Sum = 0;
		}

		if(command[0] == 'c')
		{
			MYDRON.PID_Rool.Angle_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
		}
		if(command[0] == 'e')
		{
			MYDRON.PID_Rool.Angular_Rate_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
		}

		if(command[0] == 'g')
		{
			MYDRON.PID_Rool.Angular_Rate_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
			MYDRON.Rool.Angular_Rate_Error_Sum = 0;
		}

		if(command[0] == 'h')
		{
			MYDRON.PID_Rool.Angular_Rate_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
		}








		if(command[0] == 'j')
			{
				MYDRON.PID_Yaw.Angle_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
			}

			if(command[0] == 'k')
			{
				MYDRON.PID_Yaw.Angle_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
				MYDRON.Yaw.Angle_Error_Sum = 0;
			}

			if(command[0] == 'l')
			{
				MYDRON.PID_Yaw.Angle_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
			}
			if(command[0] == 'm')
			{
				MYDRON.PID_Yaw.Angular_Rate_Factors[0] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
			}

			if(command[0] == 'n')
			{
				MYDRON.PID_Yaw.Angular_Rate_Factors[1] = ((((float)USART.UASRT_PID_VAL[1])-48)*100) + ((((float)USART.UASRT_PID_VAL[2])-48)*10) + ((((float)USART.UASRT_PID_VAL[3])-48)) + ((((float)USART.UASRT_PID_VAL[4])-48)/10) + ((((float)USART.UASRT_PID_VAL[5])-48)/100);
				MYDRON.Yaw.Angular_Rate_Error_Sum = 0;
			}

			if(command[0] == 'o')
			{
				MYDRON.PID_Yaw.Angular_Rate_Factors[2] = ((((float)USART.UASRT_PID_VAL[1])-48)*1000) + ((((float)USART.UASRT_PID_VAL[2])-48)*100) + ((((float)USART.UASRT_PID_VAL[3])-48)*10) + ((((float)USART.UASRT_PID_VAL[4])-48)) + ((((float)USART.UASRT_PID_VAL[5])-48)/10);
			}

	if(command[0] == 'r'){
		NVIC_SystemReset();
	}

	for(int i = 0; i < 15; i++){
		USART.UASRT_PID_VAL[i] = 0;
	}
//	for(int i = 0; i < 10; i++){
//		words[i] = 0;
//	}

}
