/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRAM __attribute__((section (".DRAM")))
#define IRAM __attribute__((section (".IRAM")))
#define RAM1 __attribute__((section (".RAM1")))

#define LED_G_1 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)
#define LED_G_0 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)

#define LED_R_1 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_0 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)

#define LED_Y_1 HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET)
#define LED_Y_0 HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET)

#define LED_5_1 HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET) // GY-87 LED
#define LED_5_0 HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET)

#define LED_6_1 HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_SET) // NRF24 LED
#define LED_6_0 HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET)

#define LED_7_1 HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_SET) // USART LED
#define LED_7_0 HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_RESET)

#define LED_uSD_1 HAL_GPIO_WritePin(uSD_LED_GPIO_Port, uSD_LED_Pin, GPIO_PIN_SET) // uSD LED
#define LED_uSD_0 HAL_GPIO_WritePin(uSD_LED_GPIO_Port, uSD_LED_Pin, GPIO_PIN_RESET)

#define ESC_POWER_1 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_SET)//1 = unucitve
#define ESC_POWER_0 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_RESET)//0 = active

#define NRF24_CSN_HIGH		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET)
#define NRF24_CSN_LOW		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET)

#define NRF24_CE_HIGH		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET)
#define NRF24_CE_LOW		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
///////// MPU6050
IRAM float ax,ay,az, gx, gy, gz;
IRAM float ax_ang,ay_ang,az_ang, gx_ang, gy_ang, gz_ang;
IRAM float accelx_cal, accely_cal, accelz_cal, gyrox_cal, gyroy_cal, gyroz_cal;
IRAM uint8_t MPU6050_IT_DATA[14];


const float Gyr_Scale = 65.5, Acc_Scale = 8192;//float Gyr_Scale = 65.5, Acc_Scale = 4096;
//////// W_filter
//float OldXs[16];
//float OldYs[16];
//float OldZs[16];
//
//float xval = 0;
//float yval = 0;
//float zval = 0;

///////// HMC5883L

IRAM float Mag_Z;
IRAM float Old_Mag_Z = 0;

#define FDP_Mag_Z_FQ 2

IRAM int16_t Mag_Offset_val;
IRAM uint8_t HMC5883L_Data_IT[6];

///////// main
IRAM int TIM_inte_SD = 0, TIM_inte = 0;
IRAM uint8_t STARTUP = 1;
IRAM uint32_t NRF_TIM_Inte = 0;

IRAM Complementary_Filter data;

///////// BMP180
IRAM uint16_t AC4, AC5, AC6;
IRAM int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
IRAM uint32_t B4, B7, UP;
IRAM int32_t temperature, pressure, UT, X1, X2, B5, B6, B3, X3;
IRAM float temp = 0, pres = 0, startpres = 0, ampritude = 0;
IRAM uint8_t BMP180_Press_IT[3], BMP180_Temp_IT[2];

/////////// nrf24
IRAM uint8_t RxData[32];
IRAM uint8_t Txcode[32];
IRAM uint8_t TxData[32];
IRAM uint8_t Rxcode[32];
RAM1 uint32_t analogmess;
//IRAM uint8_t xz[9];

IRAM uint8_t nRF24_Rx_Mode = 0;


/////// dron

IRAM struct Dron MYDRON;


IRAM struct Stack Old_Data_stack;

IRAM int16_t wanted_pitch_rx;// chcainy stan
IRAM int16_t wanted_rool_rx;
IRAM int16_t wanted_yaw_rx;
IRAM int16_t wanted_gz;

IRAM float last_wanted_pitch_rx = 0;// chcainy stan
IRAM float last_wanted_rool_rx = 0;
IRAM float last_wanted_yaw_rx = 0;

		#define FDP_FQ 2

IRAM int16_t wanted_thrust = 0;
IRAM int16_t thrust_limit = 10000;


IRAM double thrust_values;

IRAM float wanted_pitch = 0;// chcainy stan
IRAM float wanted_rool = 0;
IRAM float wanted_yaw = 0;

		const float wanted_rool_factro = 0.65;//95 //- 99
		const float wanted_pitch_factro = 0.66;// ok              35 / -35
		const float wanted_yaw_factro = 0.6;//95//-101

////////Filtry


IRAM uint16_t FDP_D_Gain_AR = 0;
IRAM uint16_t FDP_D_Gain = 0;

//////// PID

IRAM float now_pitch = 0;// oktualny stan
IRAM float now_rool = 0;
IRAM float now_yaw = 0;

IRAM float old_error_pitch = 0;// ruznica
IRAM float old_error_rool = 0;
IRAM float old_error_yaw = 0;

IRAM float old_error_angular_rate_pitch = 0;// ruznica
IRAM float old_error_angular_rate_rool = 0;
IRAM float old_error_angular_rate_yaw = 0;

IRAM int32_t error_sum_pitch = 0;
IRAM int32_t error_sum_rool = 0;
IRAM int32_t error_sum_yaw = 0;

IRAM int32_t error_sum_angular_rate_pitch = 0;
IRAM int32_t error_sum_angular_rate_rool = 0;
IRAM int32_t error_sum_angular_rate_yaw = 0;

IRAM float pid_pitch;
IRAM float pid_yaw;
IRAM float pid_rool;

IRAM float pid_angular_rate_pitch;
IRAM float pid_angular_rate_yaw;
IRAM float pid_angular_rate_rool;


///////////////////////////////////////////////////////////////////////////
IRAM float p_pitchfactor = 6;
IRAM float p_roolfactor = 5;
IRAM float p_yawfactor = 0;
///////////////////////////////////////////////////////////////////////////
IRAM float i_pitchfactor = 8;
IRAM float i_roolfactor = 10;
IRAM float i_yawfactor = 0;
///////////////////////////////////////////////////////////////////////////
IRAM float d_pitchfactor = 25;
IRAM float d_roolfactor = 0;
IRAM float d_yawfactor = 0;
///////////////////////////////////////////////////////////////////////////
IRAM float PID_FAC_Pitch[5];
IRAM float PID_FAC_Rool[5];
IRAM float PID_FAC_Yaw[5];


///////////////////////////////////////////////////////////////////////////
IRAM float p_angular_rate_pitchfactor = 10;
IRAM float p_angular_rate_roolfactor = 10;
IRAM float p_angular_rate_yawfactor = 15;
///////////////////////////////////////////////////////////////////////////
IRAM float i_angular_rate_pitchfactor = 0;
IRAM float i_angular_rate_roolfactor = 0;
IRAM float i_angular_rate_yawfactor = 0;
///////////////////////////////////////////////////////////////////////////
IRAM float d_angular_rate_pitchfactor = 30;
IRAM float d_angular_rate_roolfactor = 20;
IRAM float d_angular_rate_yawfactor = 5;
///////////////////////////////////////////////////////////////////////////
IRAM float PID_FAC_Angular_Rate_Pitch[5];
IRAM float PID_FAC_Angular_Rate_Rool[5];
IRAM float PID_FAC_Angular_Rate_Yaw[5];


const float looptime = 0.001;
// ESC

IRAM uint16_t SPEED1 = min_speed;
IRAM uint16_t SPEED2 = min_speed;
IRAM uint16_t SPEED3 = min_speed;
IRAM uint16_t SPEED4 = min_speed;

IRAM uint16_t OLD_SPEED1 = min_speed;
IRAM uint16_t OLD_SPEED2 = min_speed;
IRAM uint16_t OLD_SPEED3 = min_speed;
IRAM uint16_t OLD_SPEED4 = min_speed;

const uint16_t DRON_SLOWFALING = 3000;
IRAM uint8_t DRON_ON_GRUND;


///////////// USART

IRAM uint8_t UASRT_PID_VAL[15];
IRAM char command[1];
IRAM uint8_t words[10];

IRAM uint8_t commandready = 0;
IRAM uint8_t command_ch_num = 0;
IRAM uint8_t Received;

///////////// uSD
RAM1 FATFS fs;
RAM1 FRESULT fresult;
RAM1 FIL fil;
RAM1 UINT br, bw;

IRAM uint32_t Mainloop_Number = 0;
IRAM uint32_t SD_In_Use = 0;
RAM1 uint8_t DataToSendBuffer[129000];//129
IRAM uint8_t SD_enable_Flag = 0;
IRAM float wobble_strenght = 1;
IRAM uint8_t MPU6050_IRQ = 0, HMC583L_IRQ = 0, BMP180_IRQ = 0;




IRAM int i = 0, loopnum = 0;
IRAM int16_t wanted_roll_v = 0;
IRAM int16_t wanted_pitch_v = 0;
IRAM int16_t wanted_yaw_v = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

//	for(int i = 0; i < 1000000; i++){
//		//oczekiwanie az napiecia na zasilaczu narosnie
//	}
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C5_Init();
  MX_SPI6_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	  ESC_POWER_1;

  	  STARTUP = 1;

  	DRON_ON_GRUND = 1;

  	PID_FAC_Pitch[0] = p_pitchfactor;
  	PID_FAC_Pitch[1] = i_pitchfactor;
  	PID_FAC_Pitch[2] = d_pitchfactor;
  	PID_FAC_Pitch[4] = 0;

  	PID_FAC_Rool[0] = p_roolfactor;
  	PID_FAC_Rool[1] = i_roolfactor;
  	PID_FAC_Rool[2] = d_roolfactor;
  	PID_FAC_Rool[4] = 0;

  	PID_FAC_Yaw[0] = p_yawfactor;
  	PID_FAC_Yaw[1] = i_yawfactor;
  	PID_FAC_Yaw[2] = d_yawfactor;
  	PID_FAC_Yaw[4] = 0;


  	ax_ang =0,ay_ang = 0,az_ang = 0, gx_ang = 0, gy_ang = 0, gz_ang = 0;
  	Mag_Z = 0;
  	TIM_inte_SD = 0, TIM_inte = 0;
  	NRF_TIM_Inte = 0;
  	temp = 0, pres = 0, startpres = 0, ampritude = 0;
  	last_wanted_pitch_rx = 0;
  	last_wanted_rool_rx = 0;
  	last_wanted_yaw_rx = 0;
  	wanted_thrust = 0;
  	thrust_limit = 10000;


  	FDP_D_Gain_AR = 10000;
  	FDP_D_Gain = 10000;


  	old_error_pitch = 0;// ruznica
  	old_error_rool = 0;
  	old_error_yaw = 0;
  	old_error_angular_rate_pitch = 0;// ruznica
  	old_error_angular_rate_rool = 0;
  	old_error_angular_rate_yaw = 0;
  	error_sum_pitch = 0;
  	error_sum_rool = 0;
  	error_sum_yaw = 0;
  	error_sum_angular_rate_pitch = 0;
  	error_sum_angular_rate_rool = 0;
  	error_sum_angular_rate_yaw = 0;
  	SPEED1 = min_speed;
  	SPEED2 = min_speed;
  	SPEED3 = min_speed;
  	SPEED4 = min_speed;
  	OLD_SPEED1 = min_speed;
  	OLD_SPEED2 = min_speed;
  	OLD_SPEED3 = min_speed;
  	OLD_SPEED4 = min_speed;
  	commandready = 0;
  	command_ch_num = 0;
  	Mainloop_Number = 0;
  	SD_In_Use = 0;
  	wobble_strenght = 1;
  	MPU6050_IRQ = 0, HMC583L_IRQ = 0, BMP180_IRQ = 0;
  	i = 0, loopnum = 0;

  	thrust_values = 0;

	  	p_angular_rate_pitchfactor = 14;
		p_angular_rate_roolfactor = 12;
		p_angular_rate_yawfactor = 5;// -> 10
		///////////////////////////////////////////////////////////////////////////
		i_angular_rate_pitchfactor = 10;
		i_angular_rate_roolfactor = 10;
		i_angular_rate_yawfactor = 0;
		///////////////////////////////////////////////////////////////////////////
		d_angular_rate_pitchfactor = 30;
		d_angular_rate_roolfactor = 20;
		d_angular_rate_yawfactor = 5;

		p_pitchfactor = 5.5;
		p_roolfactor = 6;//5
		p_yawfactor = 5;
		///////////////////////////////////////////////////////////////////////////
		i_pitchfactor = 9;//8
		i_roolfactor = 8;//10
		i_yawfactor = 0;
		///////////////////////////////////////////////////////////////////////////
		d_pitchfactor = 30;//25
		d_roolfactor = 20;
		d_yawfactor = 0;

  	  PID_FAC_Angular_Rate_Pitch[0] = p_angular_rate_pitchfactor;
  	  PID_FAC_Angular_Rate_Pitch[1] = i_angular_rate_pitchfactor;
  	  PID_FAC_Angular_Rate_Pitch[2] = d_angular_rate_pitchfactor;
  	  PID_FAC_Angular_Rate_Rool[0] = p_angular_rate_roolfactor;//
  	  PID_FAC_Angular_Rate_Rool[1] = i_angular_rate_roolfactor;
  	  PID_FAC_Angular_Rate_Rool[2] = d_angular_rate_roolfactor;
  	  PID_FAC_Angular_Rate_Yaw[0] = p_angular_rate_yawfactor;//
  	  PID_FAC_Angular_Rate_Yaw[1] = i_angular_rate_yawfactor;
  	  PID_FAC_Angular_Rate_Yaw[2] = d_angular_rate_yawfactor;
  	  PID_FAC_Pitch[0] = p_pitchfactor;
  	  PID_FAC_Pitch[1] = i_pitchfactor;
  	  PID_FAC_Pitch[2] = d_pitchfactor;
  	  PID_FAC_Rool[0] = p_roolfactor;//
  	  PID_FAC_Rool[1] = i_roolfactor;
  	  PID_FAC_Rool[2] = d_roolfactor;
  	  PID_FAC_Yaw[0] = p_yawfactor;//
  	  PID_FAC_Yaw[1] = i_yawfactor;
  	  PID_FAC_Yaw[2] = d_yawfactor;

  	wanted_pitch_rx = 0;// chcainy stan
  	wanted_rool_rx = 0;
  	wanted_yaw_rx = 0;
  	wanted_gz = 0 ;


  	data.ox = 0;
  	data.x = 0;
  	data.oy = 0;
  	data.y = 0;
  	data.oz = 0;
  	data.z = 0;


  	pid_pitch = 0;
  	pid_yaw = 0;
  	pid_rool = 0;

  	pid_angular_rate_pitch = 0;
  	pid_angular_rate_yaw = 0;
  	pid_angular_rate_rool = 0;

  	MYDRON.PITCH_STA = 0;
  	MYDRON.ROOL_STA = 0;
  	MYDRON.YAW_STA = 0;

  	ax = 0,ay = 0,az = 0, gx = 0, gy = 0, gz = 0;
  	ax_ang = 0,ay_ang = 0,az_ang = 0, gx_ang = 0, gy_ang = 0, gz_ang = 0;
  	accelx_cal = 0, accely_cal = 0, accelz_cal = 0, gyrox_cal = 0, gyroy_cal = 0, gyroz_cal = 0;

  	nRF24_Rx_Mode = 0;

  	now_pitch = 0;
  	now_rool = 0;
  	now_yaw = 0;
  	wanted_pitch = 0;


  	uint8_t o[3] = "Odb";
  	uint8_t n[3] = "Nad";


  	LED_5_1;
  	LED_6_1;
  	LED_7_1;
  	LED_uSD_1;
  	LED_G_1;
  	LED_Y_1;
  	LED_R_1;
  	HAL_Delay(1000);
  	LED_G_0;
  	LED_Y_0;
  	LED_R_0;
  	LED_5_0;
  	LED_6_0;
  	LED_7_0;
  	LED_uSD_0;


  	for(int i = 0; i < 4000; i++){
  		Old_Data_stack.olddata[i] = 0;
  	}
  	Old_Data_stack.start_pointer = 0;
  	Old_Data_stack.end_pointer = 4000;

  	analogmess = 0;


  	HAL_TIM_Base_Start(&htim8);
  	HAL_ADC_Start_DMA(&hadc2, &analogmess, 1);
  	LED_R_1;
  	while(analogmess == 0){

  	}
  	LED_R_0;

  	Get_batteryvalue();

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//pwm do diodt RGB
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  	RGB_LED_For_BAT(MYDRON.batterysize);
  	if(MYDRON.dron_status.Battery == DRON_BATTERY_RUN_OUT)
  	{
  		LED_R_1;
  		while(1){

  		}
  	}
  	if(MYDRON.dron_status.Battery == DRON_BATTERY_CRIT_VAL){
  		LED_R_1;
  		while(1){
  		}
  	}


  /////////////////////////////// uSD
	SD_enable_Flag = HAL_GPIO_ReadPin(uSD_Detection_GPIO_Port, uSD_Detection_Pin);// jezeli karta SD jest wlozona, pin CardDetect jest zwierany do masy

	if(SD_enable_Flag == 1){
		SD_enable_Flag = 0;
	}
	else{
		SD_enable_Flag = 1;
	}

	if(SD_enable_Flag == 1){
		LED_uSD_1;
		fresult = f_mount(&fs, "/", 1);
		fresult = f_unlink("/file.txt");//skasowanie poprzedniego pliku
		fresult = f_mount(&fs, "/", 1);
		fresult = f_open(&fil, "file.txt", FA_CREATE_ALWAYS | FA_WRITE);// utwozenie nowego pliku
		LED_uSD_0;

		LED_uSD_1;
		for(int i = 0; i < 129000; i++){
			DataToSendBuffer[i] = 49;
		}
		fresult = f_write(&fil, DataToSendBuffer, 129, &bw);//64000
		fresult = f_close(&fil);

		if(fresult != FR_OK){
			while(1){

			}
		}
		LED_uSD_0;
	}

	/////////////////////////////// MPU6050
		LED_5_1;
		if(MPU6050_INIT(&hi2c5) == 0){
			LED_R_1;
			while(1){
			}
		}
		LED_Y_1;
		MPU6050_CALIBRATION(&accelx_cal, &accely_cal, &accelz_cal, &gyrox_cal, &gyroy_cal, &gyroz_cal, Gyr_Scale, Acc_Scale);
		LED_Y_0;

	/////////////////////////////// BMP180
		if(BMP180_init(&hi2c5) == 0){
			LED_R_1;
			while(1){
			}
		}
		LED_Y_1;
		BMP180_CALIBRATION(&startpres);
		LED_Y_0;
	/////////////////////////////// HMC5883L
		if(HMC5883L_Init(&hi2c5) == 0){
			LED_R_1;
			while(1){
			}
		}
		LED_Y_1;
		Mag_Offset_val = HMC5883L_Calibration();
		LED_Y_0;

		LED_5_0;



		MYDRON.dron_status.Connection = DRON_CONNECTED;
		MYDRON.dron_status.position = DRON_POSITION_OK;


		MYDRON.THRUST = 0;
		MYDRON.PITCH = 0;
		MYDRON.ROOL = 0;
		MYDRON.YAW = 0;



	/////////////////////////////// nRF24
		LED_6_1;
		nRF24_Init(&hspi6);
		nRF24_SetRXAddress(0, o);
		nRF24_SetTXAddress(n);
		nRF24_Inittest();
		nRF24_TX_Mode();


		for(int i = 0; i < 32; i++){
			Txcode[i] = 0;
		}
		Txcode[22] = 'd';
		Txcode[23] = 'r';
		Txcode[24] = 'o';
		Txcode[25] = 'n';
		Txcode[26] = '2';
		Txcode[27] = 'c';
		Txcode[28] = '1';
		Txcode[29] = 'z';
		Txcode[30] = 'a';
		Txcode[31] = '7';



		nRF24_WriteTXPayload(Txcode);
		nRF24_WaitTX();
		nRF24_RX_Mode();

		uint8_t cunter = 0;

		while(nRF24_IsBitSetInFifoStatus(NRF24_RX_EMPTY) == 1){
			LED_G_1;
			LED_Y_1;
			HAL_Delay(1);
			LED_G_0;
			LED_Y_0;

			cunter++;
			if(cunter == 1000){
				NVIC_SystemReset();
			}
		}

		nRF24_ReadRXPaylaod(RxData);
		LED_6_0;
		for(int i = 12; i < 22; i++){
			Rxcode[i] = RxData[i];
		}
	///////////////////////////////////////////////////////////////////////
		ESC_INT(&htim3);

		LED_7_1;
		HAL_UART_Receive_IT(&huart1, &Received, 1);
		LED_7_0;

		HAL_TIM_Base_Start_IT(&htim2); // przerwanie co 1 ms



	  	RGB_LED_For_BAT(MYDRON.batterysize);


	  	if(MYDRON.dron_status.Battery == DRON_BATTERY_RUN_OUT)
	  	{
	  		LED_R_1;
	  		while(1){

	  		}
	  	}



	  	if(MYDRON.dron_status.Battery == DRON_BATTERY_CRIT_VAL){
	  		LED_R_1;
	  		while(1){
	  		}
	  	}



		STARTUP = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(MYDRON.dron_status.Battery == DRON_BATTERY_RUN_OUT){
	  			LED_R_1;
	  			thrust_limit = MYDRON.THRUST;
	  		}
	  		if(MYDRON.dron_status.Battery == DRON_BATTERY_OK){
	  			thrust_limit = 10000;
	  		}
	  		if(MYDRON.dron_status.Battery == DRON_BATTERY_CRIT_VAL){
	  			ESC_POWER_1;
	  			LED_R_1;
	  			f_close(&fil);
	  			while(MYDRON.dron_status.Battery == DRON_BATTERY_CRIT_VAL){
	  				HAL_Delay(10);
	  				Get_batteryvalue();
	  			}
	  		}

	  		if(commandready == 1){
	  			LED_7_1;
	  			interpretcommand();
	  			executecommand(command, UASRT_PID_VAL);
	  			LED_7_0;

	  			PID_FAC_Pitch[0] = p_pitchfactor;
	  			PID_FAC_Pitch[1] = i_pitchfactor;
	  			PID_FAC_Pitch[2] = d_pitchfactor;

	  			PID_FAC_Angular_Rate_Pitch[0] = p_angular_rate_pitchfactor;
	  			PID_FAC_Angular_Rate_Pitch[1] = i_angular_rate_pitchfactor;
	  			PID_FAC_Angular_Rate_Pitch[2] = d_angular_rate_pitchfactor;

	  			PID_FAC_Rool[0] = p_roolfactor;
	  			PID_FAC_Rool[1] = i_roolfactor;
	  			PID_FAC_Rool[2] = d_roolfactor;

	  			PID_FAC_Angular_Rate_Rool[0] = p_angular_rate_roolfactor;
	  			PID_FAC_Angular_Rate_Rool[1] = i_angular_rate_roolfactor;
	  			PID_FAC_Angular_Rate_Rool[2] = d_angular_rate_roolfactor;

	  			PID_FAC_Yaw[0] = p_yawfactor;
	  			PID_FAC_Yaw[1] = i_yawfactor;
	  			PID_FAC_Yaw[2] = d_yawfactor;

	  			PID_FAC_Angular_Rate_Yaw[0] = p_angular_rate_yawfactor;
	  			PID_FAC_Angular_Rate_Yaw[1] = i_angular_rate_yawfactor;
	  			PID_FAC_Angular_Rate_Yaw[2] = d_angular_rate_yawfactor;
	  		}

	  		if((TIM_inte_SD == 1) && (fresult == FR_OK) && (SD_enable_Flag == 1)){// 1ms, 1KHz
	  			TIM_inte_SD = 0;

	  			uSD_Card_SendData_To_Buffer(Mainloop_Number);

	  			if(Mainloop_Number == 999){//zapisywanie karty raz na 1 sec
	  				SD_In_Use = 1;
	  				LED_uSD_1;
	  				fresult = f_open(&fil, "file.txt", FA_OPEN_APPEND | FA_WRITE);
	  				fresult = f_write(&fil, DataToSendBuffer, 129000, &bw);
	  				fresult = f_close(&fil);
	  				LED_uSD_0;
	  				SD_In_Use = 0;
	  				}

	  			Mainloop_Number = Mainloop_Number < 1000 ? Mainloop_Number+1 : 0;
//	  			if(Mainloop_Number < 1000){
//	  				Mainloop_Number++;
//	  			}
//	  			else{
//	  				Mainloop_Number = 0;
//	  			}
	  		}

	  		if(TIM_inte == 1){
	  			LED_R_0;
	  			TIM_inte = 0;
	  		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 22;
  PeriphClkInitStruct.PLL2.PLL2N = 192;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)// 1 ms
	{
		TIM_inte_SD = 1;
		TIM_inte = 1;
		NRF_TIM_Inte++;


		if(i == 0){// na calosc 100ms
			LED_5_1;
			BMP180_start_measurment_temp_IT();
			LED_5_0;


			RGB_LED_For_BAT(MYDRON.batterysize);
		}

		if(i == 10){//2
			LED_5_1;
			BMP180_start_measurment_pres_IT();
			LED_5_0;
		}

		if(i == 36){
			LED_5_1;
			HMC5883L_Get_Z_Start_IT();
			LED_5_0;
		}

		if(i == 38){
			LED_6_1;
			nRF24_TX_Mode();
			nRF24_Rx_Mode = 0;
			LED_6_0;
		}
		if(i == 40){//5
			ampritude = startpres - pres;

			MYDRON.dronheight = (uint16_t)BMP180_GET_height();
			convert_value_to_array(MYDRON.dronheight, TxData, 0, 3);

			Get_batteryvalue();

			convert_value_to_array(MYDRON.batterysize, TxData, 3, 6);

			for(int i = 0; i < 10; i++){
				TxData[22+i] = Txcode[22+i];
			}
			LED_6_1;
			nRF24_WriteTXPayload(TxData);
			NRF24_CE_HIGH;
			LED_6_0;
		}
		if(i == 41){
			NRF24_CE_LOW;
		}
		if(i >= 42){
			if(nRF24_Rx_Mode != 1){
				if(((nRF24_ReadStatus() & (1<<NRF24_MAX_RT)) || (nRF24_ReadStatus() & (1<<NRF24_TX_DS)))){
					nRF24_RX_Mode();
					nRF24_Rx_Mode = 1;
				}
			}
		}

		LED_5_1;
		MPU6050_GET_ACCANDGYR_CALANDSCL_IT();
		LED_5_0;

		i = (i == 100) ? 0 : i+1;

//		if(i == 100){
//			i = 0;
//		}
//		else{
//			i++;
//		}
		if(NRF_TIM_Inte >= 1000){
			LED_R_1;
			MYDRON.dron_status.Connection = DRON_DISCONNECTED;
		}
		if(MYDRON.dron_status.Connection == DRON_DISCONNECTED){
			wanted_rool = 0;
			wanted_pitch = 0;
			wanted_yaw = 0;
			wanted_thrust = DRON_SLOWFALING;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_15)
	{
		if(STARTUP == 0){
			LED_6_1;
			nRF24_ReadRXPaylaod(RxData);
			LED_6_0;


			LED_Y_1;

			loopnum = 0;
			for(int abc = 0; abc < 10; abc++){//sprawdzenia poprawnosci kodu nadanego
				if(RxData[12+abc] == Rxcode[abc+12]){
					loopnum++;
				}
			}

			if(loopnum == 10){
				convert_array_to_value(RxData, &wanted_pitch_v, 0, 2);//pitch
				convert_array_to_value(RxData, &wanted_roll_v, 3, 5);// pid_angular_rate_rool wanted_roll_v
				if(wobble_strenght == 1){
					convert_array_to_value(RxData, &wanted_thrust, 6, 8);//
				}
				convert_array_to_value(RxData, &wanted_yaw_v, 9, 11);//
				MYDRON.dron_status.Connection = DRON_CONNECTED;

				if(RxData[23] == 1){
					ESC_POWER_1;
					LED_R_1;
					if(SD_enable_Flag == 1){
						f_close(&fil);
						fresult = f_mount(NULL, "/", 1);
					}

				}

				if(wanted_thrust == 9999){
					MYDRON.THRUST_flag = THRUST_MAX;
	  			}
				wanted_thrust = wanted_thrust * 20;
				wanted_thrust = wanted_thrust - 10000;
				if(wanted_thrust < 0){
					wanted_thrust = 0;
	  			}

				wanted_rool_rx = (wanted_roll_v - 500)*wanted_rool_factro;//wanted_rool_rx (-90 <-> 90)
				wanted_pitch_rx = (wanted_pitch_v - 500)*wanted_pitch_factro;// (-450 <-> 450)
				wanted_yaw_rx = (wanted_yaw_v - 500)*wanted_yaw_factro;


				wanted_rool_rx = (wanted_rool_rx >= 30) ? 30 : (wanted_rool_rx <= -30) ? -30 : wanted_rool_rx;
				wanted_pitch_rx = (wanted_pitch_rx >= 30) ? 30 : (wanted_pitch_rx <= -30) ? -30 : wanted_pitch_rx;
				wanted_yaw_rx = (wanted_yaw_rx >= 30) ? 30 : (wanted_yaw_rx <= -30) ? -30 : wanted_yaw_rx;
//				if(wanted_rool_rx >= 30){
//					wanted_rool_rx = 30;
//				}
//				if(wanted_rool_rx <= -30){
//					wanted_rool_rx = -30;
//				}
//				if(wanted_pitch_rx >= 30){
//					wanted_pitch_rx = 30;
//				}
//				if(wanted_pitch_rx <= -30){
//					wanted_pitch_rx = -30;
//				}
//				if(wanted_yaw_rx >= 30){
//					wanted_yaw_rx = 30;
//				}
//				if(wanted_yaw_rx <= -30){
//					wanted_yaw_rx = -30;
//				}


				NRF_TIM_Inte = 0;
	  		}

	  			if(loopnum > 0 && loopnum < 10){
	  				MYDRON.dron_status.Connection = DRON_CONNECTION_ERROR;
	  			}
	  			LED_Y_0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//pobieranie znakw z uart
{
	words[command_ch_num] = Received;

	if(words[command_ch_num] == '\r')
	{
		char Y = '\n';
		HAL_UART_Transmit(&huart1, (uint8_t *)&Y, 1, 100);
		Y = '\r';
		HAL_UART_Transmit(&huart1, (uint8_t *)&Y, 1, 100);
		words[command_ch_num] = 0;
		commandready = 1;
	}

	HAL_UART_Transmit_IT(&huart1, (uint8_t *)&words[command_ch_num], 1);
	HAL_UART_Receive_IT(&huart1, &Received, 1);
	command_ch_num++;

	if(commandready == 1)
	{
		command_ch_num = 0;
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(MPU6050_IRQ == 1){
			MPU6050_IRQ = 0;
			LED_G_1;

			MPU6050_GET_CALANDSCL_IT(&ax, &ay, &az, &gx, &gy, &gz, accelx_cal, accely_cal, accelz_cal, gyrox_cal, gyroy_cal, gyroz_cal, Gyr_Scale, Acc_Scale);
			MPU6050_GET_ACCEL_TO_ANGLE(ax, ay, az, &ax_ang, &ay_ang/*, &az_ang*/);
			MPU6050_GET_ACCANDGYR_FILTRED(&data, ax_ang, ay_ang, Mag_Z, gx, gy, gz);


			now_pitch = data.x;
			now_rool = data.y;
			now_yaw = data.z;


			wanted_rool = (float)wanted_rool_rx/10;
			wanted_pitch = (float)wanted_pitch_rx/10;
			wanted_yaw = 0;//(float)wanted_yaw_rx/10;
				  			/*
				  				 * FDP
				  				 */
			wanted_rool = (wanted_rool * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (last_wanted_rool_rx * (1 / (1 + (FDP_FQ * looptime))));
			wanted_pitch = (wanted_pitch * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (last_wanted_pitch_rx * (1 / (1 + (FDP_FQ * looptime))));
			wanted_yaw = (wanted_yaw * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (last_wanted_yaw_rx * (1 / (1 + (FDP_FQ * looptime))));


			last_wanted_rool_rx = wanted_rool;
			last_wanted_pitch_rx = wanted_pitch;
			last_wanted_yaw_rx = wanted_yaw;


			error_sum_pitch = (MYDRON.PITCH_STA != 0) ? error_sum_pitch : error_sum_pitch + (wanted_pitch - now_pitch);
			error_sum_rool = (MYDRON.ROOL_STA != 0) ? error_sum_rool : error_sum_rool + (wanted_rool - now_rool);
			error_sum_yaw = (MYDRON.YAW_STA != 0) ? error_sum_yaw : error_sum_yaw + (wanted_yaw - now_yaw);

			error_sum_angular_rate_pitch = (MYDRON.PITCH_STA != 0) ? error_sum_angular_rate_pitch : error_sum_angular_rate_pitch + (pid_pitch - gx);
			error_sum_angular_rate_rool = (MYDRON.ROOL_STA != 0) ? error_sum_angular_rate_rool : error_sum_angular_rate_rool + (pid_rool - gy);
			error_sum_angular_rate_yaw = (MYDRON.YAW_STA != 0) ? error_sum_angular_rate_yaw : error_sum_angular_rate_yaw + (pid_yaw - gz);


			PID_cal(&pid_pitch, PID_FAC_Pitch, 1);// angle control
			PID_cal(&pid_rool, PID_FAC_Rool, 2);
			PID_cal(&pid_yaw, PID_FAC_Yaw, 3);

			PID_cal(&pid_angular_rate_pitch, PID_FAC_Angular_Rate_Pitch, 4);// angle rate control
			PID_cal(&pid_angular_rate_rool, PID_FAC_Angular_Rate_Rool, 5);
			PID_cal(&pid_angular_rate_yaw, PID_FAC_Angular_Rate_Yaw, 6);


			old_error_pitch = wanted_pitch - now_pitch;
			old_error_rool = wanted_rool - now_rool;
			old_error_yaw = wanted_yaw - now_yaw;

			old_error_angular_rate_pitch = pid_pitch - gx;
			old_error_angular_rate_rool = pid_rool - gy;
			old_error_angular_rate_yaw = wanted_yaw - gz;


			//MYDRON.ROOL 	= ((pid_angular_rate_rool > -5000) && (pid_angular_rate_rool < 5000)) ? pid_angular_rate_rool: (pid_angular_rate_rool > 0) ? 5000: -5000;
			MYDRON.ROOL 	= (pid_angular_rate_rool > 5000) ? ROOL_MAX_VAL(): (pid_angular_rate_rool < -5000) ? ROOL_MIN_VAL(): ROOL_GOOD_VAL();
			MYDRON.PITCH 	= (pid_angular_rate_pitch > 5000) ? PITCH_MAX_VAL(): (pid_angular_rate_pitch < -5000) ? PITCH_MIN_VAL(): PITCH_GOOD_VAL();
			MYDRON.YAW 		= (pid_angular_rate_yaw > 5000) ? YAW_MAX_VAL(): (pid_angular_rate_yaw < -5000) ? YAW_MIN_VAL(): YAW_GOOD_VAL();


		/*
			MYDRON.PITCH 	= (pid_angular_rate_pitch);
			MYDRON.YAW 		= (pid_angular_rate_yaw);
		 */

//			wobble_strenght = Wobble_Detect();
//			if(wobble_strenght > 1){
//				wanted_thrust = MYDRON.THRUST/wobble_strenght;
//				Wobble_handler();
//			}


			Thrust_filter(1);
			if(MYDRON.THRUST > thrust_limit){
				MYDRON.THRUST = thrust_limit;
			}


			Stack_Push(WartoscBezwgledna(PID_FAC_Pitch[4]));
			Stack_Push(WartoscBezwgledna(PID_FAC_Rool[4]));
			Stack_Push(now_pitch);
			Stack_Push(now_rool);


			SPEED1 = (((uint32_t)((MYDRON.THRUST*0.7) + MYDRON.ROOL - MYDRON.PITCH + MYDRON.YAW + min_speed + 500)) < max_speed) ? ((MYDRON.THRUST*0.7) + MYDRON.ROOL - MYDRON.PITCH + MYDRON.YAW + min_speed + 500) : max_speed;//trust 7000 max
			SPEED2 = (((uint32_t)((MYDRON.THRUST*0.7) - MYDRON.ROOL - MYDRON.PITCH - MYDRON.YAW + min_speed + 500)) < max_speed) ? ((MYDRON.THRUST*0.7) - MYDRON.ROOL - MYDRON.PITCH - MYDRON.YAW + min_speed + 500) : max_speed;//
			SPEED3 = (((uint32_t)((MYDRON.THRUST*0.7) + MYDRON.ROOL + MYDRON.PITCH - MYDRON.YAW + min_speed + 500)) < max_speed) ? ((MYDRON.THRUST*0.7) + MYDRON.ROOL + MYDRON.PITCH - MYDRON.YAW + min_speed + 500) : max_speed;//
			SPEED4 = (((uint32_t)((MYDRON.THRUST*0.7) - MYDRON.ROOL + MYDRON.PITCH + MYDRON.YAW + min_speed + 500)) < max_speed) ? ((MYDRON.THRUST*0.7) - MYDRON.ROOL + MYDRON.PITCH + MYDRON.YAW + min_speed + 500) : max_speed;//

			if(SPEED1 != OLD_SPEED1){
				ESC_1_SPEED(SPEED1);
			}
			if(SPEED2 != OLD_SPEED2){
				ESC_2_SPEED(SPEED2);
			}
			if(SPEED3 != OLD_SPEED3){
				ESC_3_SPEED(SPEED3);
			}
			if(SPEED4 != OLD_SPEED4){
				ESC_4_SPEED(SPEED4);
			}

			OLD_SPEED1 = SPEED1;
			OLD_SPEED2 = SPEED2;
			OLD_SPEED3 = SPEED3;
			OLD_SPEED4 = SPEED4;

			LED_G_0;
	}
	if(HMC583L_IRQ == 1){
		HMC583L_IRQ = 0;
		Mag_Z = (float)(HMC5883L_Get_Z_End_IT() - Mag_Offset_val);
		Mag_Z = (Mag_Z * (FDP_Mag_Z_FQ * 0.1) / (1 + (FDP_Mag_Z_FQ * 0.1))) + (Old_Mag_Z * (1 / (1 + (FDP_Mag_Z_FQ * 0.1)))); // 0.1 to looptime, co 100ms odczyt
		Old_Mag_Z = Mag_Z;
	}
	if(BMP180_IRQ == 1){
		temperature = BMP180_GET_temp_IT();
		temp = BMP180_GET_temp(temperature);
		BMP180_IRQ = 0;
	}
	if(BMP180_IRQ == 2){
		pressure = BMP180_GET_pres_IT();
		pres = BMP180_GET_pres(pressure);
		BMP180_IRQ = 0;
	}
}


void convert_array_to_value(uint8_t arrayfrom[], int16_t *value , uint8_t rangebegin, uint8_t rangeend){
	*value = 0;
	int range = rangeend - rangebegin;

	for(int y = 0; y < range+1; y++){
		*value = *value + arrayfrom[rangebegin+y]*pow(10, range - y);
	}

}


void convert_value_to_array(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;
	for(int i = 0; i < range+1; i++){// 3
		while(value >= (uint16_t)pow(10,range - i)){
			if(value == 0){
				break;
			}
			value -= (uint16_t)potenga(10,range - i);
			x++;
		}
		arraytoputin[rangebegin+loopnum] = (uint8_t)x;
		loopnum++;
		x = 0;
	}
}
uint32_t potenga(int a, int b){
	int32_t c = a;
	if(b == 0){
		return 1;
	}
	if(b == 1){
		return a;
	}
	if(b > 1){
			for(int i = 1; i < b; i++){
			a = a*c;
		}
		return a;
	}
	if(b < 0){
		for(int i = 0; i < b; i++){
			a = a/c;
		}
		return a;
	}
	return a;
}
float WartoscBezwgledna(float a){
	a = (a < 0) ? a*(-1) : a;
	return a;
//	if(a < 0){
//		return a*(-1);
//	}
//	else{
//		return a;
//	}
}



int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void RGB_LED_Set_color(uint8_t R, uint8_t G, uint8_t B){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, B);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, R);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, G);
}

void RGB_LED_For_BAT(uint8_t batval){
/*
 * batval == 150  r = 0, g = 255 b = 0
 * batval == 75	  r = 128 g = 128 b = 0
 * batval == 0    r = 255 g = 0 b = 0
 *
 *   r + g = 255
 */
	RGB_LED_Set_color((255 - ((float)batval*2.55)), ((float)batval*2.55), 0);
}

/*convert_value_to_array2
 *
 *
 * value - wartosc konwertowana
 * arraytoputin - adres tablicy do której chcemy zapisac
 * rangebegin - poczontek tablicy w którym zaczynamy zapisywac
 * rangeend - koniec tablicy
 *
 * nalezy pamientac by dodac +1 do rangeend, poniewaz pierwsze miejsce w tablicy moze byc znak -
 *
 *	[0] -
 *	[1] 2
 *	[2] 2
 */

void convert_value_to_array2(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;

	if(value < 0){
		arraytoputin[rangebegin] = '-';
	}

	for(int i = 1; i < range+1; i++){// 3
		while(value >= (uint16_t)potenga(10,range - i)){
			if(value == 0){
				break;
			}
			value -= (uint16_t)potenga(10,range - i);
			x++;
		}
		arraytoputin[rangebegin + loopnum] = (uint8_t)x + 48;//zamiana na ASCII
		loopnum++;
		x = 0;
	}
}

/*
 * 12.055
 *
 * 1
 * 2
 * .
 * 0
 * 5
 * 5
 *
 *	1000.1
 *
 *	range = 5
 *
 */
void convert_value_to_array3(float value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;
	int power_of_value = 0;
	float a;
	int kropka;


	if(value < 0){
		arraytoputin[rangebegin] = '-';
		value = value * -1;
	}

	//sprawdzenie wagi pierwsazej liczby znaczącej
	//potrzebne do dzielenia
	for(int i = 1; i < range*2; i++){
		a = pow(10,range - i);
		if(value >= a){
			break;
		}
		power_of_value++;
	}

	power_of_value = range - power_of_value - 1;

	//		Sprawdzic gdzie jest kropka 0.00123 123.01 12.12
	if(power_of_value <= 0){
		arraytoputin[rangebegin + 1] = '.';
		kropka = 1;
	}
	if(power_of_value > 0){
		arraytoputin[rangebegin + power_of_value + 1] = '.';
		kropka = power_of_value + 1;
	}


	value = value*pow(10,(range-1) - power_of_value);


	for(int i = 0; i < range-2; i++){// range-2 poniewaz jest znak kropki i ewentualny znak minusa na początku
		while(value >= (uint16_t)pow(10,(range-1) - i)){
			if(value <= 0){
				break;
			}
			value = value - pow(10,(range-1) - i);
			x++;
		}

        if(loopnum == kropka){
            loopnum++;
        }
		arraytoputin[rangebegin + loopnum] = (uint8_t)x + 48;//zamiana na ASCII
		loopnum++;
		x = 0;
	}
}
/*
 *
 * Save to SD :
 *
 * SPEED1	5 6
 * SPEED2 	5
 * SPEED3	5
 * SPEED4	5
 *
 * error_sum_pitch 7 8
 * error_sum_rool 7
 * error_sum_yaw 7
 *
 * data.x 6 7
 * data.y 6
 * data.z 6
 *
 * MYDRON.batterysize 3 4
 * MYDRON.dron_status.Connection 1 2
 *
 * 					 PID_FAC_Pitch[5]; 7 8
 *						 PID_FAC_Rool[5]; 7
 *						 						PID_FAC_Yaw[5]; 7
 *
 * Numer pentli 7 8
 *
 * Mag_Z 3
 */
void uSD_Card_SendData_To_Buffer(uint32_t a){
	convert_value_to_array2(Mainloop_Number, DataToSendBuffer, (0 + (128*a)), (8 + (128*a)));
	 DataToSendBuffer[(8 + (128*a))] = ' ';


	convert_value_to_array2(SPEED1, DataToSendBuffer, (9 + (128*a)), (15 + (128*a)));
	 DataToSendBuffer[(15 + (128*a))] = ' ';
	convert_value_to_array2(SPEED2, DataToSendBuffer, (16 + (128*a)), (22 + (128*a)));
	 DataToSendBuffer[(22 + (128*a))] = ' ';
	convert_value_to_array2(SPEED3, DataToSendBuffer, (23 + (128*a)), (29 + (128*a)));
	 DataToSendBuffer[(29 + (128*a))] = ' ';
	convert_value_to_array2(SPEED4, DataToSendBuffer, (30 + (128*a)), (36 + (128*a)));
	 DataToSendBuffer[(36 + (128*a))] = ' ';

	convert_value_to_array3(wanted_rool, DataToSendBuffer, (37 + (128*a)), (45 + (128*a)));
	 DataToSendBuffer[(45 + (128*a))] = ' ';
	convert_value_to_array3(wanted_pitch, DataToSendBuffer, (46 + (128*a)), (54 + (128*a)));
	 DataToSendBuffer[(54 + (128*a))] = ' ';
	convert_value_to_array3(wanted_yaw, DataToSendBuffer, (55 + (128*a)), (63 + (128*a)));
	 DataToSendBuffer[(63 + (128*a))] = ' ';

	convert_value_to_array2(error_sum_pitch, DataToSendBuffer, (64 + (128*a)), (72 + (128*a)));
	 DataToSendBuffer[(72 + (128*a))] = ' ';
	convert_value_to_array2(error_sum_rool, DataToSendBuffer, (73 + (128*a)), (81 + (128*a)));
	 DataToSendBuffer[(81 + (128*a))] = ' ';
	convert_value_to_array2(error_sum_yaw, DataToSendBuffer, (82 + (128*a)), (90 + (128*a)));
	 DataToSendBuffer[(90 + (128*a))] = ' ';

	convert_value_to_array3(data.x, DataToSendBuffer, (91 + (128*a)), (98 + (128*a)));
	 DataToSendBuffer[(98 + (128*a))] = ' ';
	convert_value_to_array3(data.y, DataToSendBuffer, (99 + (128*a)), (106 + (128*a)));
	 DataToSendBuffer[(106 + (128*a))] = ' ';
	convert_value_to_array3(data.z, DataToSendBuffer, (107 + (128*a)), (114 + (128*a)));
	 DataToSendBuffer[(114 + (128*a))] = ' ';


	convert_value_to_array2(MYDRON.batterysize, DataToSendBuffer, (115 + (128*a)), (119 + (128*a)));
	 DataToSendBuffer[(119 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.dron_status.Connection, DataToSendBuffer, (120 + (128*a)), (122 + (128*a)));
	 DataToSendBuffer[(122 + (128*a))] = ' ';
	convert_value_to_array2(Mag_Z, DataToSendBuffer, (123 + (128*a)), (127 + (128*a)));
	 DataToSendBuffer[(128 + (128*a))] = '\n';
}

void Stack_Push(float data){
	Old_Data_stack.start_pointer++;
	if(Old_Data_stack.start_pointer == 4000){
		Old_Data_stack.start_pointer = 0;
	}
	Old_Data_stack.olddata[Old_Data_stack.start_pointer] = data;

	Old_Data_stack.end_pointer++;
	if(Old_Data_stack.start_pointer == 4000){
		Old_Data_stack.start_pointer = 0;
	}
}

int16_t ROOL_MAX_VAL(void){
	MYDRON.ROOL_STA = 2;
	return 5000;
}
int16_t ROOL_MIN_VAL(void){
	MYDRON.ROOL_STA = 1;
	return -5000;
}
int16_t PITCH_MAX_VAL(void){
	MYDRON.PITCH_STA = 2;
	return 5000;
}
int16_t PITCH_MIN_VAL(void){
	MYDRON.PITCH_STA = 1;
	return -5000;
}
int16_t YAW_MAX_VAL(void){
	MYDRON.YAW_STA = 2;
	return 5000;
}
int16_t YAW_MIN_VAL(void){
	MYDRON.YAW_STA = 1;
	return -5000;
}
int16_t ROOL_GOOD_VAL(void){
	MYDRON.ROOL_STA = 0;
	return pid_angular_rate_rool;
}
int16_t PITCH_GOOD_VAL(void){
	MYDRON.PITCH_STA = 0;
	return pid_angular_rate_pitch;
}
int16_t YAW_GOOD_VAL(void){
	MYDRON.YAW_STA = 0;
	return pid_angular_rate_yaw;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
