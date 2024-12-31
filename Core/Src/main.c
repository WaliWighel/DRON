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
IRAM MPU6050_Struct MPU6050;
IRAM Complementary_Filter data;
IRAM BMP180_Struct BMP180;
IRAM NRF24_Struct NRF24;
IRAM Dron MYDRON;
IRAM Stack Old_Data_stack;
IRAM HMC5883L_Struct HMC5883L;

///////// main
IRAM int TIM_inte_SD = 0, TIM_inte = 0;
IRAM uint8_t STARTUP = 1;
IRAM uint32_t NRF_TIM_Inte = 0;
RAM1 uint32_t analogmess;
IRAM uint32_t i = 0, loopnum = 0;
IRAM uint8_t NRF24_Messages_SC = 0;
IRAM uint8_t NRF24_inte = 0;
IRAM uint16_t FDP_D_Gain_AR = 0;
IRAM uint16_t FDP_D_Gain = 0;
const float looptime = 0.001;
IRAM float wobble_strenght = 1;
const uint16_t DRON_SLOWFALING = 3000;
IRAM uint8_t DRON_ON_GRUND;


#define FDP_Mag_Z_FQ 2
#define FDP_FQ 2

///////////// USART

IRAM uint8_t UASRT_PID_VAL[15];//todo
IRAM char command[1];
IRAM uint8_t words[10];

IRAM uint8_t commandready = 0;
IRAM uint8_t command_ch_num = 0;
IRAM uint8_t Received;

///////////// uSD
RAM1 FATFS fs;//todo
RAM1 FRESULT fresult;
RAM1 FIL fil;
RAM1 UINT br, bw;

IRAM uint32_t Mainloop_Number = 0;
IRAM uint32_t SD_In_Use = 0;
RAM1 uint8_t DataToSendBuffer[129000];//129
IRAM uint8_t SD_enable_Flag = 0;
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
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  	  ESC_POWER_1;


  	STARTUP = 1;
  	DRON_ON_GRUND = 1;

  	MPU6050.Acc.Acc_Scale = 8192;
  	MPU6050.Acc.acc_x_cal = 0;
  	MPU6050.Acc.acc_y_cal = 0;
  	MPU6050.Acc.acc_z_cal = 0;
  	MPU6050.Acc.ax_ang = 0;
  	MPU6050.Acc.ay_ang = 0;
  	MPU6050.Acc.az_ang = 0;
  	MPU6050.Acc.ax = 0;
  	MPU6050.Acc.ay = 0;
  	MPU6050.Acc.az = 0;
  	MPU6050.Gyr.Gyr_Scale = 65.5;
  	MPU6050.Gyr.gyr_x_cal = 0;
  	MPU6050.Gyr.gyr_y_cal = 0;
  	MPU6050.Gyr.gyr_z_cal = 0;
  	MPU6050.Gyr.gx = 0;
  	MPU6050.Gyr.gy = 0;
  	MPU6050.Gyr.gz = 0;
  	MPU6050.MPU6050_IRQ = 0;



  	data.x = 0;
  	data.y = 0;
  	data.z = 0;
  	data.ox = 0;
  	data.oy = 0;
  	data.oz = 0;



	NRF24.NRF24_MODE = NRF24_Error;
	NRF24.Status = NRF24_Ready;
	NRF24.Message_Status = 0;
	NRF24.NRF24_Message_count = 0;
	NRF24.Timer_1 = 0;
	NRF24.Timer_2 = 0;
	NRF24.Step = 0;
	NRF24.SPI_Rx_Inte = 0;
	NRF24.SPI_Tx_Inte = 0;



	BMP180.BMP180_IRQ = 0;
	BMP180.ampritude = 0;
	BMP180.pres = 0;
	BMP180.startpres = 0;
	BMP180.temp = 0;
	BMP180.Raw_Data.pressure = 0;
	BMP180.Raw_Data.temperature = 0;



	MYDRON.dronheight = 0;
	MYDRON.batterysize = 0;

	MYDRON.Pitch.Angle_Error_Sum = 0;
	MYDRON.Pitch.Angle_Error = 0;
	MYDRON.Pitch.Angular_Rate_Error = 0;
	MYDRON.Pitch.Angular_Rate_Error_Sum = 0;
	MYDRON.Pitch.Last_Wanted_rx = 0;
	MYDRON.Pitch.Now = 0;
	MYDRON.Pitch.Old_Angle_Error = 0;
	MYDRON.Pitch.Old_Angular_Rate_Error = 0;
	MYDRON.Pitch.Value = 0;
	MYDRON.Pitch.Wanted = 0;
	MYDRON.Pitch.Wanted_Factor = 0.65;
	MYDRON.Pitch.Wanted_rx = 0;
	MYDRON.Pitch.Wanted_v = 0;

	MYDRON.Rool.Angle_Error_Sum = 0;
	MYDRON.Rool.Angle_Error = 0;
	MYDRON.Rool.Angular_Rate_Error = 0;
	MYDRON.Rool.Angular_Rate_Error_Sum = 0;
	MYDRON.Rool.Last_Wanted_rx = 0;
	MYDRON.Rool.Now = 0;
	MYDRON.Rool.Old_Angle_Error = 0;
	MYDRON.Rool.Old_Angular_Rate_Error = 0;
	MYDRON.Rool.Value = 0;
	MYDRON.Rool.Wanted = 0;
	MYDRON.Rool.Wanted_Factor = 0.66;
	MYDRON.Rool.Wanted_rx = 0;
	MYDRON.Rool.Wanted_v = 0;

	MYDRON.Yaw.Angle_Error_Sum = 0;
	MYDRON.Yaw.Angle_Error = 0;
	MYDRON.Yaw.Angular_Rate_Error = 0;
	MYDRON.Yaw.Angular_Rate_Error_Sum = 0;
	MYDRON.Yaw.Last_Wanted_rx = 0;
	MYDRON.Yaw.Now = 0;
	MYDRON.Yaw.Old_Angle_Error = 0;
	MYDRON.Yaw.Old_Angular_Rate_Error = 0;
	MYDRON.Yaw.Value = 0;
	MYDRON.Yaw.Wanted = 0;
	MYDRON.Yaw.Wanted_Factor = 0.6;
	MYDRON.Yaw.Wanted_rx = 0;
	MYDRON.Yaw.Wanted_v = 0;

	MYDRON.PID_Pitch.Angle_Factors[0] = 6;
	MYDRON.PID_Pitch.Angle_Factors[1] = 8;
	MYDRON.PID_Pitch.Angle_Factors[2] = 25;
	MYDRON.PID_Pitch.Angle_Factors[3] = 0;
	MYDRON.PID_Pitch.Angle_Factors[4] = 0;
	MYDRON.PID_Pitch.Angular_Rate_Factors[0] = 10;
	MYDRON.PID_Pitch.Angular_Rate_Factors[1] = 0;
	MYDRON.PID_Pitch.Angular_Rate_Factors[2] = 30;
	MYDRON.PID_Pitch.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Pitch.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Pitch.Angle_Value = 0;
	MYDRON.PID_Pitch.Angular_Rate_Value = 0;

	MYDRON.PID_Rool.Angle_Factors[0] = 5;
	MYDRON.PID_Rool.Angle_Factors[1] = 10;
	MYDRON.PID_Rool.Angle_Factors[2] = 0;
	MYDRON.PID_Rool.Angle_Factors[3] = 0;
	MYDRON.PID_Rool.Angle_Factors[4] = 0;
	MYDRON.PID_Rool.Angular_Rate_Factors[0] =  10;
	MYDRON.PID_Rool.Angular_Rate_Factors[1] = 0;
	MYDRON.PID_Rool.Angular_Rate_Factors[2] = 20;
	MYDRON.PID_Rool.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Rool.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Rool.Angle_Value = 0;
	MYDRON.PID_Rool.Angular_Rate_Value = 0;

	MYDRON.PID_Yaw.Angle_Factors[0] = 10;
	MYDRON.PID_Yaw.Angle_Factors[1] = 0;
	MYDRON.PID_Yaw.Angle_Factors[2] = 0;
	MYDRON.PID_Yaw.Angle_Factors[3] = 0;
	MYDRON.PID_Yaw.Angle_Factors[4] = 0;
	MYDRON.PID_Yaw.Angular_Rate_Factors[0] = 15;
	MYDRON.PID_Yaw.Angular_Rate_Factors[1] = 0;
	MYDRON.PID_Yaw.Angular_Rate_Factors[2] = 5;
	MYDRON.PID_Yaw.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Yaw.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Yaw.Angle_Value = 0;
	MYDRON.PID_Yaw.Angular_Rate_Value = 0;

	MYDRON.Thrust.Now = 0;
	MYDRON.Thrust.Old_Speed_1 = min_speed;
	MYDRON.Thrust.Old_Speed_2 = min_speed;
	MYDRON.Thrust.Old_Speed_3 = min_speed;
	MYDRON.Thrust.Old_Speed_4 = min_speed;
	MYDRON.Thrust.Speed_1 = min_speed;
	MYDRON.Thrust.Speed_2 = min_speed;
	MYDRON.Thrust.Speed_3 = min_speed;
	MYDRON.Thrust.Speed_4 = min_speed;
	MYDRON.Thrust.Thrust_Limit = 10000;
	MYDRON.Thrust.Values = 0;
	MYDRON.Thrust.Wanted = 0;

	MYDRON.Status.Battery = DRON_BATTERY_OK;
	MYDRON.Status.Code = DRON_CODE_OK;
	MYDRON.Status.Connection = DRON_CONNECTED;
	MYDRON.Status.Wobble = NO_WOBBLE;



	HMC5883L.Directions.Heading = 0;
	HMC5883L.Directions.Old_X = 0;
	HMC5883L.Directions.Old_Y = 0;
	HMC5883L.Directions.Old_Z = 0;
	HMC5883L.Directions.X = 0;
	HMC5883L.Directions.Y = 0;
	HMC5883L.Directions.Z = 0;
	HMC5883L.HMC583L_IRQ = 0;
	HMC5883L.Off_Set_Values.X = 0;
	HMC5883L.Off_Set_Values.Y = 0;
	HMC5883L.Off_Set_Values.Z = 0;



  	TIM_inte_SD = 0, TIM_inte = 0;
  	NRF_TIM_Inte = 0;
  	FDP_D_Gain_AR = 0;
  	FDP_D_Gain = 0;
  	commandready = 0;
  	command_ch_num = 0;
  	Mainloop_Number = 0;
  	SD_In_Use = 0;
  	wobble_strenght = 1;
  	i = 0, loopnum = 0;
  	NRF24_inte = 0;


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
  	if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT)
  	{
  		LED_R_1;
  		while(1){

  		}
  	}
  	if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
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
			NVIC_SystemReset();
			while(1){

			}
		}
		LED_uSD_0;
	}

	/////////////////////////////// MPU6050
		LED_5_1;
		if(MPU6050_INIT(&hi2c5) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		MPU6050_CALIBRATION();
		LED_Y_0;

	/////////////////////////////// BMP180
		if(BMP180_init(&hi2c5) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		BMP180_CALIBRATION(&BMP180.startpres);
		LED_Y_0;
	/////////////////////////////// HMC5883L
		if(HMC5883L_Init(&hi2c5) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		HMC5883L.Off_Set_Values.Z = HMC5883L_Calibration();
		LED_Y_0;

		LED_5_0;



	/////////////////////////////// nRF24
		LED_6_1;
		nRF24_Init(&hspi1);
		nRF24_SetRXAddress(0, o);
		nRF24_SetTXAddress(n);
		nRF24_TX_Mode();
		NRF24.NRF24_MODE = NRF24_Tx_Mode;



		for(int i = 0; i < 32; i++){
			NRF24.Txcode[i] = 0;
		}
		NRF24.Txcode[22] = 'd';
		NRF24.Txcode[23] = 'r';
		NRF24.Txcode[24] = 'o';
		NRF24.Txcode[25] = 'n';
		NRF24.Txcode[26] = '2';
		NRF24.Txcode[27] = 'c';
		NRF24.Txcode[28] = '1';
		NRF24.Txcode[29] = 'z';
		NRF24.Txcode[30] = 'a';
		NRF24.Txcode[31] = '7';



		nRF24_WriteTXPayload(NRF24.Txcode);
		nRF24_WaitTX();
		nRF24_RX_Mode();
		NRF24.NRF24_MODE = NRF24_Rx_Mode;

		uint8_t cunter = 0;

		while(nRF24_IsBitSetInFifoStatus(NRF24_RX_EMPTY) == 1){
			LED_G_1;
			LED_Y_1;
			HAL_Delay(1);
			LED_G_0;
			LED_Y_0;

			cunter++;
			if(cunter == 200){
				NVIC_SystemReset();
			}
		}

		while(NRF24_inte != 1){

		}
		nRF24_ReadRXPaylaod(NRF24.RxData);
		STARTUP = 0;
		LED_6_0;
		for(int i = 12; i < 22; i++){
			NRF24.Rxcode[i] = NRF24.RxData[i];
		}
	///////////////////////////////////////////////////////////////////////
		HAL_TIM_Base_Start_IT(&htim2); // przerwanie co 1 ms

		ESC_INT(&htim3);

		LED_7_1;
		HAL_UART_Receive_IT(&huart1, &Received, 1);
		LED_7_0;

//		HAL_TIM_Base_Start_IT(&htim2); // przerwanie co 1 ms

	  	RGB_LED_For_BAT(MYDRON.batterysize);

	  	if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT)
	  	{
	  		LED_R_1;
	  		while(1){

	  		}
	  	}

	  	if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
	  		LED_R_1;
	  		while(1){
	  		}
	  	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT){
	  			LED_R_1;
	  			MYDRON.Thrust.Thrust_Limit = MYDRON.Thrust.Now;
	  		}
	  		if(MYDRON.Status.Battery == DRON_BATTERY_OK){
	  			MYDRON.Thrust.Thrust_Limit = 10000;
	  		}
	  		if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
	  			ESC_POWER_1;
	  			LED_R_1;
	  			f_close(&fil);
	  			while(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
	  				HAL_Delay(10);
	  				Get_batteryvalue();
	  			}
	  		}

	  		if(commandready == 1){
	  			LED_7_1;
	  			interpretcommand();
	  			executecommand(command, UASRT_PID_VAL);
	  			LED_7_0;
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
	  		}


			if(i == 0){
				LED_5_1;
				BMP180_start_measurment_temp_IT();
				LED_5_0;
			}
			if(i == 5){
				LED_5_1;
				BMP180_READ_temp_IT();
				LED_5_0;
			}
			if(BMP180.Timer == 0 && BMP180.BMP180_IRQ == 3){
				LED_5_1;
				BMP180_start_measurment_pres_IT();
				LED_5_0;
				BMP180.BMP180_IRQ = 0;
				BMP180.Timer = 8;
			}
			if(BMP180.Timer == 0 && BMP180.I2C_Tx_IRQ == 2){
				LED_5_1;
				BMP180_READ_pres_IT();
				LED_5_0;
			}


	  		if(MYDRON.Status.Connection == DRON_DISCONNECTED){
	  			if(NRF24.Step == 0){
	  				NRF24.Step = 2;
	  				NRF24.Status = NRF24_Ready;
	  				HAL_Delay(10);
	  			}
	  			if(NRF24.Step == 9){
	  				NRF24.Step = 0;

	  			}
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
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

		if(NRF24.Timer_1 == 0){
			if(NRF24.Step == 2  && NRF24.Status == NRF24_Ready){
				LED_6_1;
				nRF24_TX_Mode();
				LED_6_0;
				NRF24.Step++;//Step 3
				NRF24.Timer_1 = 1;
			}
			if(NRF24.Step == 3 && NRF24.Timer_1 == 0){
				NRF24.Status = NRF24_Ready;
				NRF24.NRF24_MODE = NRF24_Tx_Mode;
				NRF24.TxData[10] = (MYDRON.Status.Connection == DRON_DISCONNECTED) ? 1: 0;

				LED_6_1;
				nRF24_WriteTXPayload_IT(NRF24.TxData);
				LED_6_0;
				NRF24.Step++;//Step 4
			}
			if(NRF24.Step == 5){
				NRF24_CE_LOW;
				NRF24.Step++;//Step 6
			}
			if(NRF24.Step == 6){
				LED_6_1;
				uint8_t status = nRF24_ReadStatus();
				if(((status & (1<<NRF24_MAX_RT)) || (status & (1<<NRF24_TX_DS)))){//nRF24_WaitTX()
					NRF24.Timer_2 = 1;
					NRF24.Status = NRF24_Ready;
					NRF24.Step++;//Step 7
				}
				LED_6_0;
			}
		}
		if(NRF24.Timer_2 == 0 && NRF24.Step == 7 && NRF24.Status == NRF24_Ready){
			LED_6_1;
			nRF24_RX_Mode();
			LED_6_0;
			NRF24.Timer_2 = 1;
			NRF24.Step++;//Step 8
		}
		if(NRF24.Timer_2 == 0 && NRF24.Step == 8){
			NRF24.Status = NRF24_Ready;
			NRF24.NRF24_MODE = NRF24_Rx_Mode;
			NRF24.Step = 0;
		}


		if(i == 0){// na calosc 100ms


			RGB_LED_For_BAT(MYDRON.batterysize);
		}
//		if(i == 9){
//
//		}
//		if(i == 10){//2
//			LED_5_1;
//			BMP180_READ_temp_IT();
//			LED_5_0;
//		}


		if(i == 36){
			LED_5_1;
			HMC5883L_Get_Z_Start_IT();
			LED_5_0;
		}

		if(i == 40){//5
			//BMP180_READ_pres_IT();
			BMP180.ampritude = BMP180.startpres - BMP180.pres;

			MYDRON.dronheight = (int16_t)BMP180_GET_height();
			convert_value_to_array(MYDRON.dronheight, NRF24.TxData, 0, 3);

			Get_batteryvalue();

			convert_value_to_array(MYDRON.batterysize, NRF24.TxData, 3, 6);

			for(int i = 0; i < 10; i++){
				NRF24.TxData[22+i] = NRF24.Txcode[22+i];
			}
		}


		LED_5_1;
		MPU6050_GET_ACCANDGYR_CALANDSCL_IT();
		LED_5_0;

		i = (i == 100) ? 0 : i+1;

		if(NRF_TIM_Inte >= 1000){
			LED_R_1;
			MYDRON.Status.Connection = DRON_DISCONNECTED;
		}
		if(MYDRON.Status.Connection == DRON_DISCONNECTED){
			MYDRON.Rool.Wanted = 0;
			MYDRON.Pitch.Wanted = 0;
			MYDRON.Yaw.Wanted = 0;
			MYDRON.Thrust.Wanted = DRON_SLOWFALING;
		}
		(NRF24.Timer_1 != 0) ? NRF24.Timer_1--: 0;
		(NRF24.Timer_2 != 0) ? NRF24.Timer_2--: 0;
		(BMP180.Timer != 0) ? BMP180.Timer--: 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_15)
	{
		NRF24_inte = 1;
		if(STARTUP == 0){
			LED_6_1;
			nRF24_ReadRXPaylaod_IT(NRF24.RxData);
			NRF24.Step++;//step 1
			LED_6_0;
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

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
//	if(BMP180.I2C_Tx_IRQ == 1){
//		BMP180_READ_temp_IT();
//		BMP180.I2C_Tx_IRQ = 0;
//	}
//	if(BMP180.I2C_Tx_IRQ == 2){
//		BMP180_READ_pres_IT();
//		BMP180.I2C_Tx_IRQ = 0;
//	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(MPU6050.MPU6050_IRQ == 1){
			MPU6050.MPU6050_IRQ = 0;

			LED_G_1;

			MPU6050_GET_CALANDSCL_IT();
			MPU6050_GET_ACCEL_TO_ANGLE();
			MPU6050_GET_ACCANDGYR_FILTRED(&data, HMC5883L.Directions.Z);



			MYDRON.Pitch.Now = data.x;
			MYDRON.Rool.Now = data.y;
			MYDRON.Yaw.Now = data.z;


			MYDRON.Rool.Wanted = (float)MYDRON.Rool.Wanted_rx/10;
			MYDRON.Pitch.Wanted = (float)MYDRON.Pitch.Wanted_rx/10;
			MYDRON.Yaw.Wanted = 0;//(float)MYDRON.Yaw.Wanted_rx/10;
				  			/*
				  				 * FDP
				  				 */
			MYDRON.Rool.Wanted = (MYDRON.Rool.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Rool.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));
			MYDRON.Pitch.Wanted = (MYDRON.Pitch.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Pitch.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));
			MYDRON.Yaw.Wanted = (MYDRON.Yaw.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Yaw.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));


			MYDRON.Rool.Last_Wanted_rx = MYDRON.Rool.Wanted;
			MYDRON.Pitch.Last_Wanted_rx = MYDRON.Pitch.Wanted;
			MYDRON.Yaw.Last_Wanted_rx= MYDRON.Yaw.Wanted;


			MYDRON.Pitch.Angle_Error = MYDRON.Pitch.Wanted - MYDRON.Pitch.Now;
			MYDRON.Rool.Angle_Error = MYDRON.Rool.Wanted - MYDRON.Rool.Now;
		  	MYDRON.Yaw.Angle_Error = MYDRON.Yaw.Wanted - MYDRON.Yaw.Now;
		  	MYDRON.Pitch.Angular_Rate_Error = MYDRON.PID_Pitch.Angle_Value- MPU6050.Gyr.gx;
		  	MYDRON.Rool.Angular_Rate_Error = MYDRON.PID_Rool.Angle_Value- MPU6050.Gyr.gy;
		  	MYDRON.Yaw.Angular_Rate_Error = MYDRON.PID_Yaw.Angle_Value- MPU6050.Gyr.gz;


			MYDRON.Pitch.Angle_Error_Sum = (MYDRON.PID_Pitch.Status != 0) ? MYDRON.Pitch.Angle_Error_Sum : MYDRON.Pitch.Angle_Error_Sum + (MYDRON.Pitch.Angle_Error);//pitch_error -> pitch_error
			MYDRON.Rool.Angle_Error_Sum = (MYDRON.PID_Rool.Status != 0) ? MYDRON.Rool.Angle_Error_Sum : MYDRON.Rool.Angle_Error_Sum + (MYDRON.Rool.Angle_Error);//rool_error
			MYDRON.Yaw.Angle_Error_Sum = (MYDRON.PID_Yaw.Status != 0) ? MYDRON.Yaw.Angle_Error_Sum : MYDRON.Yaw.Angle_Error_Sum + (MYDRON.Yaw.Angle_Error);//yaw_error

			MYDRON.Pitch.Angular_Rate_Error_Sum = (MYDRON.PID_Pitch.Status != 0) ? MYDRON.Pitch.Angular_Rate_Error_Sum : MYDRON.Pitch.Angular_Rate_Error_Sum + (MYDRON.Pitch.Angular_Rate_Error);//pitch_ar_error
			MYDRON.Rool.Angular_Rate_Error_Sum = (MYDRON.PID_Rool.Status != 0) ? MYDRON.Rool.Angular_Rate_Error_Sum : MYDRON.Rool.Angular_Rate_Error_Sum + (MYDRON.Rool.Angular_Rate_Error);
			MYDRON.Yaw.Angular_Rate_Error_Sum = (MYDRON.PID_Yaw.Status != 0) ? MYDRON.Yaw.Angular_Rate_Error_Sum : MYDRON.Yaw.Angular_Rate_Error_Sum + (MYDRON.Yaw.Angular_Rate_Error);


			PID_call(MYDRON);


			MYDRON.Pitch.Old_Angle_Error = MYDRON.Pitch.Angle_Error;
			MYDRON.Rool.Old_Angle_Error = MYDRON.Rool.Angle_Error;
			MYDRON.Yaw.Old_Angle_Error = MYDRON.Yaw.Angle_Error;

			MYDRON.Pitch.Old_Angle_Error = MYDRON.Pitch.Angular_Rate_Error;
			MYDRON.Rool.Old_Angle_Error = MYDRON.Rool.Angular_Rate_Error;
			MYDRON.Yaw.Old_Angle_Error = MYDRON.Yaw.Angular_Rate_Error;


			MYDRON.Pitch.Value  	= (MYDRON.PID_Pitch.Angular_Rate_Value > 5000) ? PITCH_MAX_VAL(): (MYDRON.PID_Pitch.Angular_Rate_Value < -5000) ? PITCH_MIN_VAL(): PITCH_GOOD_VAL();
			MYDRON.Rool.Value 	    = (MYDRON.PID_Rool.Angular_Rate_Value > 5000) ? ROOL_MAX_VAL(): (MYDRON.PID_Rool.Angular_Rate_Value < -5000) ? ROOL_MIN_VAL(): ROOL_GOOD_VAL();
			MYDRON.Yaw.Value  		= (MYDRON.PID_Yaw.Angular_Rate_Value > 5000) ? YAW_MAX_VAL(): (MYDRON.PID_Yaw.Angular_Rate_Value < -5000) ? YAW_MIN_VAL(): YAW_GOOD_VAL();


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
			if(MYDRON.Thrust.Now > MYDRON.Thrust.Thrust_Limit){
				MYDRON.Thrust.Now = MYDRON.Thrust.Thrust_Limit;
			}


			Stack_Push(WartoscBezwgledna(MYDRON.PID_Pitch.Angle_Factors[4]));
			Stack_Push(WartoscBezwgledna(MYDRON.PID_Pitch.Angle_Factors[4]));
			Stack_Push(MYDRON.Pitch.Now);
			Stack_Push(MYDRON.Rool.Now);


			MYDRON.Thrust.Speed_1 = (((uint32_t)((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value - MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value - MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500) : max_speed;//trust 7000 max
			MYDRON.Thrust.Speed_2 = (((uint32_t)((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value - MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value - MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500) : max_speed;//
			MYDRON.Thrust.Speed_3 = (((uint32_t)((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value + MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value + MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500) : max_speed;//
			MYDRON.Thrust.Speed_4 = (((uint32_t)((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value + MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value + MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500) : max_speed;//

			if( MYDRON.Thrust.Speed_1 != MYDRON.Thrust.Old_Speed_1){
				ESC_1_SPEED( MYDRON.Thrust.Speed_1);
			}
			if( MYDRON.Thrust.Speed_2 != MYDRON.Thrust.Old_Speed_2){
				ESC_2_SPEED( MYDRON.Thrust.Speed_2);
			}
			if( MYDRON.Thrust.Speed_3 != MYDRON.Thrust.Old_Speed_3){
				ESC_3_SPEED( MYDRON.Thrust.Speed_3);
			}
			if( MYDRON.Thrust.Speed_4 != MYDRON.Thrust.Old_Speed_4){
				ESC_4_SPEED( MYDRON.Thrust.Speed_4);
			}

			MYDRON.Thrust.Old_Speed_1 = MYDRON.Thrust.Speed_1;
			MYDRON.Thrust.Old_Speed_2 = MYDRON.Thrust.Speed_2;
			MYDRON.Thrust.Old_Speed_3 = MYDRON.Thrust.Speed_3;
			MYDRON.Thrust.Old_Speed_4 = MYDRON.Thrust.Speed_4;

			LED_G_0;
	}
	if(HMC5883L.HMC583L_IRQ == 1){
		HMC5883L.HMC583L_IRQ = 0;
		HMC5883L_Get_Z_End_IT();
		HMC5883L.Directions.Z = (HMC5883L.Directions.Z * (FDP_Mag_Z_FQ * 0.1) / (1 + (FDP_Mag_Z_FQ * 0.1))) + (HMC5883L.Directions.Old_Z * (1 / (1 + (FDP_Mag_Z_FQ * 0.1)))); // 0.1 to looptime, co 100ms odczyt
		HMC5883L.Directions.Old_Z = HMC5883L.Directions.Z;
	}
	if(BMP180.BMP180_IRQ == 1){
		BMP180.Timer = 1;
		BMP180.Raw_Data.temperature = BMP180_GET_temp_IT();
		BMP180.temp = BMP180_GET_temp(BMP180.Raw_Data.temperature);
		BMP180.BMP180_IRQ = 3;
	}
	if(BMP180.BMP180_IRQ == 2){
		BMP180.Raw_Data.pressure = BMP180_GET_pres_IT();
		BMP180.pres = BMP180_GET_pres(BMP180.Raw_Data.pressure);
		BMP180.BMP180_IRQ = 0;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi){
	if(NRF24.SPI_Tx_Inte == 1 && NRF24.Step == 4){
		LED_6_1;
		NRF24_CSN_HIGH;
		NRF24_CE_HIGH;//nRF24_WaitTX()
		LED_6_0;
		//
		NRF24.Status = NRF24_Ready;
		NRF24.Timer_1 = 1;
		//NRF24.Message_Status = 2;
		NRF24.SPI_Tx_Inte = 0;
		NRF24.Step++;//Step 5

	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){//todo
	if(STARTUP == 0 && NRF24.SPI_Rx_Inte == 1 && NRF24.Step == 1){

		NRF24.SPI_Rx_Inte = 0;
		LED_6_1;
		nRF24_ReadRXPaylaod_IT_End();
		LED_6_0;
		NRF24.Step++;//step 2
		NRF24.Timer_1 = 2;

		LED_Y_1;

		loopnum = 0;
		for(int abc = 0; abc < 10; abc++){//sprawdzenia poprawnosci kodu nadanego
			if(NRF24.RxData[12+abc] == NRF24.Rxcode[abc+12]){
				loopnum++;
			}
		}

		if(loopnum == 10){
			convert_array_to_value(NRF24.RxData, &MYDRON.Pitch.Wanted_v, 0, 2);//pitch
			convert_array_to_value(NRF24.RxData, &MYDRON.Rool.Wanted_v, 3, 5);// pid_angular_rate_rool wanted_roll_v
			if(wobble_strenght == 1){
				convert_array_to_value(NRF24.RxData, &MYDRON.Thrust.Wanted, 6, 8);//
			}
			convert_array_to_value(NRF24.RxData, &MYDRON.Yaw.Wanted_v, 9, 11);//
			MYDRON.Status.Connection = DRON_CONNECTED;

			if(NRF24.RxData[23] == 1){
				ESC_POWER_1;
				LED_R_1;
				if(SD_enable_Flag == 1){
					f_close(&fil);
					fresult = f_mount(NULL, "/", 1);
					}
			}

			if(MYDRON.Thrust.Wanted == 9999){
				MYDRON.Thrust.Max_Flag = THRUST_MAX;
		  	}
			MYDRON.Thrust.Wanted = MYDRON.Thrust.Wanted * 20;
			MYDRON.Thrust.Wanted = MYDRON.Thrust.Wanted - 10000;
			if(MYDRON.Thrust.Wanted < 0){
				MYDRON.Thrust.Wanted = 0;
			}

			MYDRON.Rool.Wanted_rx = (MYDRON.Rool.Wanted_v - 500)*MYDRON.Rool.Wanted_Factor;//MYDRON.Rool.Wanted_rx (-90 <-> 90)
			MYDRON.Pitch.Wanted_rx = (MYDRON.Pitch.Wanted_v - 500)*MYDRON.Pitch.Wanted_Factor;// (-450 <-> 450)
			MYDRON.Yaw.Wanted_rx = (MYDRON.Yaw.Wanted_v - 500)*MYDRON.Yaw.Wanted_Factor;// wanted yaw is in deg/s

			MYDRON.Rool.Wanted_rx = (MYDRON.Rool.Wanted_rx >= 300) ? 300 : (MYDRON.Rool.Wanted_rx <= -300) ? -300 : MYDRON.Rool.Wanted_rx;
			MYDRON.Pitch.Wanted_rx = (MYDRON.Pitch.Wanted_rx >= 300) ? 300 : (MYDRON.Pitch.Wanted_rx <= -300) ? -300 : MYDRON.Pitch.Wanted_rx;
			MYDRON.Yaw.Wanted_rx = (MYDRON.Yaw.Wanted_rx >= 300) ? 300 : (MYDRON.Yaw.Wanted_rx <= -300) ? -300 : MYDRON.Yaw.Wanted_rx;

			NRF_TIM_Inte = 0;
		}

		if(loopnum > 0 && loopnum < 10){
			MYDRON.Status.Connection = DRON_CONNECTION_ERROR;
		}
		LED_Y_0;
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
 * MYDRON.Pitch.Angle_Error_pitch 7 8
 * MYDRON.Pitch.Angle_Error_rool 7
 * MYDRON.Pitch.Angle_Error_yaw 7
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


	convert_value_to_array2(MYDRON.Thrust.Speed_1, DataToSendBuffer, (9 + (128*a)), (15 + (128*a)));
	 DataToSendBuffer[(15 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Thrust.Speed_2, DataToSendBuffer, (16 + (128*a)), (22 + (128*a)));
	 DataToSendBuffer[(22 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Thrust.Speed_3, DataToSendBuffer, (23 + (128*a)), (29 + (128*a)));
	 DataToSendBuffer[(29 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Thrust.Speed_4, DataToSendBuffer, (30 + (128*a)), (36 + (128*a)));
	 DataToSendBuffer[(36 + (128*a))] = ' ';

	convert_value_to_array3(MYDRON.Rool.Wanted, DataToSendBuffer, (37 + (128*a)), (45 + (128*a)));
	 DataToSendBuffer[(45 + (128*a))] = ' ';
	convert_value_to_array3(MYDRON.Pitch.Wanted, DataToSendBuffer, (46 + (128*a)), (54 + (128*a)));
	 DataToSendBuffer[(54 + (128*a))] = ' ';
	convert_value_to_array3(MYDRON.Yaw.Wanted, DataToSendBuffer, (55 + (128*a)), (63 + (128*a)));
	 DataToSendBuffer[(63 + (128*a))] = ' ';

	convert_value_to_array2(MYDRON.Pitch.Angle_Error_Sum, DataToSendBuffer, (64 + (128*a)), (72 + (128*a)));
	 DataToSendBuffer[(72 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Rool.Angle_Error_Sum, DataToSendBuffer, (73 + (128*a)), (81 + (128*a)));
	 DataToSendBuffer[(81 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Yaw.Angle_Error_Sum, DataToSendBuffer, (82 + (128*a)), (90 + (128*a)));
	 DataToSendBuffer[(90 + (128*a))] = ' ';

	convert_value_to_array3(data.x, DataToSendBuffer, (91 + (128*a)), (98 + (128*a)));
	 DataToSendBuffer[(98 + (128*a))] = ' ';
	convert_value_to_array3(data.y, DataToSendBuffer, (99 + (128*a)), (106 + (128*a)));
	 DataToSendBuffer[(106 + (128*a))] = ' ';
	convert_value_to_array3(data.z, DataToSendBuffer, (107 + (128*a)), (114 + (128*a)));
	 DataToSendBuffer[(114 + (128*a))] = ' ';


	convert_value_to_array2(MYDRON.batterysize, DataToSendBuffer, (115 + (128*a)), (119 + (128*a)));
	 DataToSendBuffer[(119 + (128*a))] = ' ';
	convert_value_to_array2(MYDRON.Status.Connection, DataToSendBuffer, (120 + (128*a)), (122 + (128*a)));
	 DataToSendBuffer[(122 + (128*a))] = ' ';
	convert_value_to_array2(HMC5883L.Directions.Z, DataToSendBuffer, (123 + (128*a)), (127 + (128*a)));
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
	MYDRON.PID_Rool.Status = 2;
	return 5000;
}
int16_t ROOL_MIN_VAL(void){
	MYDRON.PID_Rool.Status = 1;
	return -5000;
}
int16_t PITCH_MAX_VAL(void){
	MYDRON.PID_Pitch.Status = 2;
	return 5000;
}
int16_t PITCH_MIN_VAL(void){
	MYDRON.PID_Pitch.Status = 1;
	return -5000;
}
int16_t YAW_MAX_VAL(void){
	MYDRON.PID_Yaw.Status = 2;
	return 5000;
}
int16_t YAW_MIN_VAL(void){
	MYDRON.PID_Yaw.Status = 1;
	return -5000;
}
int16_t ROOL_GOOD_VAL(void){
	MYDRON.PID_Rool.Status = 0;
	return MYDRON.PID_Rool.Angular_Rate_Value;
}
int16_t PITCH_GOOD_VAL(void){
	MYDRON.PID_Pitch.Status = 0;
	return MYDRON.PID_Pitch.Angular_Rate_Value;
}
int16_t YAW_GOOD_VAL(void){
	MYDRON.PID_Yaw.Status = 0;
	return MYDRON.PID_Yaw.Angular_Rate_Value;
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
