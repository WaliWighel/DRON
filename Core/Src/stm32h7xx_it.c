/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern float ax,ay,az, gx, gy, gz;
extern float ax_ang,ay_ang,az_ang, gx_ang, gy_ang, gz_ang;
extern float accelx_cal, accely_cal, accelz_cal, gyrox_cal, gyroy_cal, gyroz_cal;
extern uint8_t MPU6050_IT_DATA[14];


extern float Gyr_Scale, Acc_Scale;//float Gyr_Scale = 65.5, Acc_Scale = 4096;
//////// W_filter
//float OldXs[16];
//float OldYs[16];
//float OldZs[16];
//
//float xval ;
//float yval ;
//float zval ;

///////// HMC5883L

extern float Mag_Z;
extern float Old_Mag_Z ;

#define FDP_Mag_Z_FQ 0.5

extern int16_t Mag_Offset_val;
extern uint8_t HMC5883L_Data_IT[6];

///////// main
extern int TIM_inte_SD , TIM_inte ;
extern uint8_t STARTUP;
extern uint32_t NRF_TIM_Inte ;

extern Complementary_Filter data;

///////// BMP180
extern uint16_t AC4, AC5, AC6;
extern int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
extern uint32_t B4, B7, UP;
extern int32_t temperature, pressure, UT, X1, X2, B5, B6, B3, X3;
extern float temp , pres , startpres , ampritude ;
extern uint8_t BMP180_Press_IT[3], BMP180_Temp_IT[2];

/////////// nrf24
extern uint8_t RxData[32];
extern uint8_t Txcode[32];
extern uint8_t TxData[32];
extern uint8_t Rxcode[32];
extern uint32_t analogmess;
extern uint8_t xz[9];

extern uint8_t nRF24_Rx_Mode ;


/////// dron

extern struct Dron MYDRON;





extern struct Stack Old_Data_stack;

extern int16_t wanted_pitch_rx;// chcainy stan
extern int16_t wanted_rool_rx;
extern int16_t wanted_yaw_rx;
extern int16_t wanted_gz;

extern float last_wanted_pitch_rx ;// chcainy stan
extern float last_wanted_rool_rx ;
extern float last_wanted_yaw_rx ;

		#define FDP_FQ 2

extern int16_t wanted_thrust ;
extern int16_t thrust_limit;


extern double thrust_values;

extern float wanted_pitch ;// chcainy stan
extern float wanted_rool ;
extern float wanted_yaw ;

		extern  float wanted_rool_factr;//95 //- 99
		extern  float wanted_pitch_factro;// ok              35 / -35
		extern  float wanted_yaw_factro;//95//-101

////////Filtry


extern int16_t FDP_Out ;
extern int16_t FDP_D_Gain ;

//////// PID

extern float now_pitch ;// oktualny stan
extern float now_rool ;
extern float now_yaw ;

extern float old_error_pitch ;// ruznica
extern float old_error_rool ;
extern float old_error_yaw ;

extern float old_error_angular_rate_pitch ;// ruznica
extern float old_error_angular_rate_rool ;
extern float old_error_angular_rate_yaw ;

extern int32_t error_sum_pitch ;
extern int32_t error_sum_rool ;
extern int32_t error_sum_yaw ;

extern int32_t error_sum_angular_rate_pitch ;
extern int32_t error_sum_angular_rate_rool ;
extern int32_t error_sum_angular_rate_yaw ;

extern float pid_pitch;
extern float pid_yaw;
extern float pid_rool;

extern float pid_angular_rate_pitch;
extern float pid_angular_rate_yaw;
extern float pid_angular_rate_rool;


///////////////////////////////////////////////////////////////////////////
extern float p_pitchfactor;
extern float p_roolfactor;
extern float p_yawfactor ;
///////////////////////////////////////////////////////////////////////////
extern float i_pitchfactor;
extern float i_roolfactor;
extern float i_yawfactor ;
///////////////////////////////////////////////////////////////////////////
extern float d_pitchfactor;
extern float d_roolfactor ;
extern float d_yawfactor ;
///////////////////////////////////////////////////////////////////////////
extern float PID_FAC_Pitch[5];
extern float PID_FAC_Rool[5];
extern float PID_FAC_Yaw[5];


///////////////////////////////////////////////////////////////////////////
extern float p_angular_rate_pitchfactor;
extern float p_angular_rate_roolfactor;
extern float p_angular_rate_yawfactor;
///////////////////////////////////////////////////////////////////////////
extern float i_angular_rate_pitchfactor ;
extern float i_angular_rate_roolfactor ;
extern float i_angular_rate_yawfactor ;
///////////////////////////////////////////////////////////////////////////
extern float d_angular_rate_pitchfactor;
extern float d_angular_rate_roolfactor;
extern float d_angular_rate_yawfactor;
///////////////////////////////////////////////////////////////////////////
extern float PID_FAC_Angular_Rate_Pitch[5];
extern float PID_FAC_Angular_Rate_Rool[5];
extern float PID_FAC_Angular_Rate_Yaw[5];


extern float looptime;
// ESC

extern uint16_t SPEED1 ;
extern uint16_t SPEED2 ;
extern uint16_t SPEED3 ;
extern uint16_t SPEED4 ;

extern uint16_t OLD_SPEED1;
extern uint16_t OLD_SPEED2;
extern uint16_t OLD_SPEED3;
extern uint16_t OLD_SPEED4;

extern uint16_t DRON_SLOWFALING;
extern uint8_t DRON_ON_GRUND;


///////////// USART

extern uint8_t UASRT_PID_VAL[15];
extern char command[1];
extern uint8_t words[10];

extern uint8_t commandready ;
extern uint8_t command_ch_num ;
extern uint8_t Received;

///////////// uSD
extern FATFS fs;
extern FRESULT fresult;
extern FIL fil;
extern UINT br, bw;

extern uint32_t Mainloop_Number ;
extern uint32_t SD_In_Use ;
extern uint8_t DataToSendBuffer[129000];//129
extern uint8_t SD_enable_Flag ;
extern float wobble_strenght;
extern uint8_t MPU6050_IRQ , HMC583L_IRQ , BMP180_IRQ ;




extern int i , loopnum ;
extern int16_t wanted_roll_v ;
extern int16_t wanted_pitch_v ;
extern int16_t wanted_yaw_v ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t FatFsCnt = 0;
volatile uint32_t Timer1, Timer2;

void SDTimer_Handler(void)//1ms
{
  if(Timer1 > 0)
    Timer1--;

  if(Timer2 > 0)
    Timer2--;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c5;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */


	LED_G_0;
	LED_Y_0;
	LED_R_1;
//todo create crash log
	f_close(&fil);

	wanted_rool = 0;
	wanted_pitch = 0;
	wanted_yaw = 0;
	wanted_thrust = DRON_SLOWFALING;
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
	   	HAL_Delay(1);
		MPU6050_GET_ACCANDGYR_CALANDSCL(&ax, &ay, &az, &gx, &gy, &gz, accelx_cal, accely_cal, accelz_cal, gyrox_cal, gyroy_cal, gyroz_cal, Gyr_Scale, Acc_Scale);
		MPU6050_GET_ACCEL_TO_ANGLE(ax, ay, az, &ax_ang, &ay_ang/*, &az_ang*/);
		MPU6050_GET_ACCANDGYR_FILTRED(&data, ax_ang, ay_ang, Mag_Z, gx, gy, gz);


		now_pitch = data.x;
		now_rool = data.y;
		now_yaw = data.z;


		last_wanted_rool_rx = wanted_rool;
		last_wanted_pitch_rx = wanted_pitch;
		last_wanted_yaw_rx = wanted_yaw;


		error_sum_pitch = error_sum_pitch + (wanted_pitch - now_pitch);
		error_sum_rool = error_sum_rool + (wanted_rool - now_rool);
		error_sum_yaw = error_sum_yaw + (wanted_yaw - now_yaw);

		error_sum_angular_rate_pitch = error_sum_angular_rate_pitch + (pid_pitch - gx);
		error_sum_angular_rate_rool = error_sum_angular_rate_rool + (pid_rool - gy);
		error_sum_angular_rate_yaw = error_sum_angular_rate_yaw + (pid_yaw - gz);


		PID_cal(&pid_pitch, PID_FAC_Pitch, 1);// angle control
		PID_cal(&pid_rool, PID_FAC_Rool, 2);

		PID_cal(&pid_angular_rate_pitch, PID_FAC_Angular_Rate_Pitch, 4);// angle rate control
		PID_cal(&pid_angular_rate_rool, PID_FAC_Angular_Rate_Rool, 5);
		PID_cal(&pid_angular_rate_yaw, PID_FAC_Angular_Rate_Yaw, 6);


		old_error_pitch = wanted_pitch - now_pitch;
		old_error_rool = wanted_rool - now_rool;

		old_error_angular_rate_pitch = pid_pitch - gx;
		old_error_angular_rate_rool = pid_rool - gy;
		old_error_angular_rate_yaw = wanted_yaw - gz;


		MYDRON.ROOL = pid_angular_rate_rool;
		MYDRON.PITCH = pid_angular_rate_pitch;
		MYDRON.YAW = pid_angular_rate_yaw;



		Thrust_filter(1);
		if(MYDRON.THRUST > thrust_limit){
			MYDRON.THRUST = thrust_limit;
		}


		SPEED1 = (MYDRON.THRUST*0.7)+ MYDRON.ROOL - MYDRON.PITCH + MYDRON.YAW + min_speed + 500;//trust 7000 max
		SPEED2 = (MYDRON.THRUST*0.7)- MYDRON.ROOL - MYDRON.PITCH - MYDRON.YAW + min_speed + 500;//
		SPEED3 = (MYDRON.THRUST*0.7)+ MYDRON.ROOL + MYDRON.PITCH - MYDRON.YAW + min_speed + 500;//
		SPEED4 = (MYDRON.THRUST*0.7)- MYDRON.ROOL + MYDRON.PITCH + MYDRON.YAW + min_speed + 500;//

		ESC_1_SPEED(SPEED1);
		ESC_2_SPEED(SPEED2);
		ESC_3_SPEED(SPEED3);
		ESC_4_SPEED(SPEED4);


		OLD_SPEED1 = SPEED1;
		OLD_SPEED2 = SPEED2;
		OLD_SPEED3 = SPEED3;
		OLD_SPEED4 = SPEED4;
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

	LED_G_1;
	LED_Y_0;
	LED_R_1;
	//todo create crash log
	f_close(&fil);
	wanted_rool = 0;
	wanted_pitch = 0;
	wanted_yaw = 0;
	wanted_thrust = DRON_SLOWFALING;
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

	LED_G_0;
	LED_Y_0;
	LED_R_1;
	//todo create crash log
	f_close(&fil);
	wanted_rool = 0;
	wanted_pitch = 0;
	wanted_yaw = 0;
	wanted_thrust = DRON_SLOWFALING;
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

	LED_G_1;
	LED_Y_1;
	LED_R_1;
	//todo create crash log
	f_close(&fil);
	wanted_rool = 0;
	wanted_pitch = 0;
	wanted_yaw = 0;
	wanted_thrust = DRON_SLOWFALING;
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

	LED_G_0;
	LED_Y_1;
	LED_R_1;
	//todo create crash log
	f_close(&fil);
	wanted_rool = 0;
	wanted_pitch = 0;
	wanted_yaw = 0;
	wanted_thrust = DRON_SLOWFALING;
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	  FatFsCnt++;
	  if(FatFsCnt >= 10)
	  {
	    FatFsCnt = 0;
	    SDTimer_Handler();
	  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(NRF24_IRQ_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles I2C5 event interrupt.
  */
void I2C5_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C5_EV_IRQn 0 */

  /* USER CODE END I2C5_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c5);
  /* USER CODE BEGIN I2C5_EV_IRQn 1 */

  /* USER CODE END I2C5_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
