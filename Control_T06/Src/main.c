/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef set_Filter_up;
CAN_TxHeaderTypeDef TxHeader;
RTC_TimeTypeDef sTime;
extern RTC_HandleTypeDef hrtc;
uint32_t TxMailbox;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*---Defines do acelerômetro---*/
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define MPU6050_ADDR 0xD0
/*-----------------------------*/

/*---Defines p/ Frontal--------*/
#define Throtle0_min 0x0001
#define Throtle0_max 0x0010
#define Throtle1_min 0x0001
#define Throtle1_max 0x0010

#define Steer_lft 0x0001
#define Steer_ctr 0x0010
#define Steer_rgt 0x0100
/*-----------------------------*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag_can = 1;
volatile uint8_t flag_tim = 1;
uint8_t dummy = 0xFF;


/*--------Análise de tempo---------*/
uint32_t T1 = 0; 
uint32_t T2 = 0;
uint32_t T = 0;
uint32_t counter = 0, Acc =0;
uint32_t min = 10000, max = 0;
double Average;
/*---------------------------------*/

/*-----Integração Acelerômetro-----*/
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
float Ax, Ay, Az, Gx, Gy, Gz;
volatile uint16_t X , Y , Z;
/*---------------------------------*/

/*-----Diferencial-----------------*/
double K, dif_lef , dif_rgt;
/*---------------------------------*/

/*-----ADC-------------------------*/
uint32_t data[1], value[2], adc_buffer[2];
/*---------------------------------*/

/*-----Leitura Frontal-------------*/
volatile uint8_t Front_Data[9];
uint8_t	PowerUp_Btn , PowerDwn_Btn , Ready_Btn;
double	Steer , Throtle0 , Throtle1 , Throtle; 
/*---------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Setup(void);
void Timer_Setup(void);
void Time_Analisys (void);
void MPU6050_init(void);
void MPU6050_Read_Accel(void);
void Differencial(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void Front_read(void);
double Saturate(double Min, double Value , double Max);
void Can_Rx_Wait(void);

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
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  //MX_WWDG_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	CAN_Setup();
	Timer_Setup();
	//MPU6050_init();
	//HAL_ADC_Start_DMA( &hadc1 , adc_buffer , 2 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//T1 = HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
		
		Can_Rx_Wait();
		Front_read();
		
		//MPU6050_Read_Accel();
		
		//Differencial();
		
		//T2 = HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
		
		//Time_Analisys ();
		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CAN_Setup(void){
	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	
//---------Configurações de filtro e máscara---------

	set_Filter_up.FilterIdHigh = 0;  
	set_Filter_up.FilterIdLow = 0; 
	set_Filter_up.FilterMaskIdHigh = 0; 					
	set_Filter_up.FilterMaskIdLow = 0; 
	set_Filter_up.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	set_Filter_up.FilterBank = 0;
	set_Filter_up.FilterMode = CAN_FILTERMODE_IDMASK;
	set_Filter_up.FilterScale = CAN_FILTERSCALE_32BIT;
	set_Filter_up.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &set_Filter_up);
	
//---------Configurações do envio--------------------
	
	
	
//---------Início do funcionamento CAN---------------	
	
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void Timer_Setup(){
	
	HAL_RTC_Init(&hrtc);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

void Time_Analisys (){
	
	counter++;
	
	T = T2 - T1;
	
	if ( T < min ){
		min = T;
	}else{
		max = T;
	}
	
	Acc += T;  
	Average = (double) Acc/counter;
	
}

void MPU6050_init(void){
	uint8_t check, Data;

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	
	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}

void Differencial(void){	
			
			/*K = (D*tan((steer*24.5)/(180/pi))) / (2*L);
			Dif_lef = (397*Throtle + 22)*(1 - K);
			Dif_rgt = (397*Throtle + 22)*(1 + K);
			Dif_lft = Saturate(22, Dif_lft, 419);
			Dif_rgt = Saturate(22, Dif_rgt, 419);*/
				
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint16_t) dif_lef);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,(uint16_t) dif_rgt);
	
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint32_t Sum;
	if (hadc -> Instance == ADC1){
		for ( int i = 0 ; i < 2 ; i++ ){
			for( int j = 0 ; j < 20 ; j++ ){
				Sum += adc_buffer[i];
			}
			value[i] = Acc/20;
			Sum = 0;
		}
	}
}*/

void Front_read(){
	
	TxHeader.StdId = 0x410;
	TxHeader.DLC = 1;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, &dummy, &TxMailbox);

	/*----------Recuperação de dados----------------*/
	Throtle0 =  ((Front_Data[1]<<8) | Front_Data[0]);
	
	Throtle1 =  ((Front_Data[3]<<8) | Front_Data[2]);
	
	Steer 	 = 	((Front_Data[5]<<8) | Front_Data[4]);
	/*----------------------------------------------*/
	
	/*-----------Definição da Throtle---------------*/
	Throtle = (double) (Throtle0 - Throtle0_min)/(Throtle0_max - Throtle0_min) ;
	Throtle = Saturate(0, Throtle, 1);
	/*----------------------------------------------*/
	
	/*-----------Definição do esterço---------------*/
	Steer = (double)(Steer - Steer_ctr); 
	if ( Steer < 0 ) {
		Steer = Steer / (Steer_rgt - Steer_ctr);
	}	else{
			Steer = Steer / (Steer_ctr - Steer_lft);
	}	
	Steer = Saturate(-1, Steer, 1);
	/*----------------------------------------------*/
	
	/*------------Dados p/ Telemetria---------------*/

	/*----------------------------------------------*/
}

double Saturate(double Min, double Value , double Max){
	double V;
	
	if ( Value < Min ){
		V = Min;
	} else {
			if ( Value > Max ){
				V = Max;
			} else {
				V = Value;
			}
		}

	return V;
}

void Can_Rx_Wait(){
	while(flag_can);
	flag_can = 1;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
