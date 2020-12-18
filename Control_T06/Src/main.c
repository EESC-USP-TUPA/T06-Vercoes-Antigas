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
#define ACCEL_YOUT_H_REG 0x3D
#define ACCEL_ZOUT_H_REG 0x3F
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define	GYRO_YOUT_H_REG 0x45
#define	GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define MPU6050_ADDR 0xD0
/*-----------------------------*/

/*---Defines p/ Frontal--------*/
#define Throtle0_min 0x0001
#define Throtle0_max 0x0010
#define Throtle1_min 0x0001
#define Throtle1_max 0x0010

#define Throtle0_short 0x0000
#define Throtle1_short 0x0000
#define Throtle0_open  0x0000
#define Throtle1_open  0x0000

#define Steer_lft 0x0001
#define Steer_ctr 0x0010
#define Steer_rgt 0x0100
/*-----------------------------*/

/*---Defines para freio---*/
#define Brake0_min 0x0001
#define Brake0_max 0x0001

#define Brake0_short 0x0000
#define Brake1_short 0x0000
#define Brake0_open  0x0000
#define Brake1_open  0x0000
/*------------------------*/

#define False 0
#define True  1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag_can = 1;
volatile uint8_t flag_tim = 1;
uint8_t dummy = 0xFF;
uint8_t flag_display = 0;


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
double Ax, Ay, Az, Gx, Gy, Gz, Gccx = 0 ,Gccy = 0, Gccz = 0, Accx = 0, Accy = 0, Accz = 0, Offx = 0 , Offy = 0 , Offz = 0, Sup_x = 0, Sup_y = 0, Sup_z = 0, Inf_x = 0, Inf_y = 0, Inf_z = 0;
float Offgx = 0,Offgy = 0, Offgz = 0, Sup_gx = 0 , Sup_gy = 0 ,Sup_gz = 0, Inf_gx = 0, Inf_gy = 0, Inf_gz = 0;
volatile uint16_t X , Y , Z;

int flag_accel = 0, flag_gyro = 0;
/*---------------------------------*/

/*-----Diferencial-----------------*/
double K, dif_lef , dif_rgt;
/*---------------------------------*/

/*-----ADC-------------------------*/
uint32_t data[1], value[2], adc_buffer[2];
uint16_t Brake0, Brake1;
uint8_t adc_buffer_count = 0;
uint32_t Acc_adc[3] = {0,0,0};
double Brake;
/*---------------------------------*/

/*-----BMS-------------------------*/
volatile uint8_t bms[2];
/*---------------------------------*/

/*-----Leitura Frontal-------------*/
volatile uint8_t Front_Data[9];
uint8_t	PowerUp_Btn , PowerDwn_Btn , Ready_Btn;
double	Steer , Throtle0 , Throtle1 , Throtle; 
/*---------------------------------*/

/*----Diferencial Simples----------*/
double K, Dif_lef, Dif_rgt;
/*---------------------------------*/

/*----Elementos de segurança-------*/
int flag_dif = 0, flag_apps = 0, flag_bse = 0, flag_bppc = 0;
/*---------------------------------*/

/*----- Leitura dos inversores ----*/
uint8_t inv[4]; 
/*---------------------------------*/

/*-------Dados telemetria----------*/

uint8_t Tel_data[32];
int flag_tel = 0;

//Tel_data[0]  -> Throtle
//Tel_data[1]  -> Steer
//Tel_data[2]  -> Counter LSB
//Tel_data[3]  -> Counter MSB
//Tel_data[4]  -> Brake
//Tel_data[5]  -> Ax LSB
//Tel_data[6]  -> Ax MSB
//Tel_data[7]  -> Ay LSB
//Tel_data[8]  -> Ay MSB
//Tel_data[9]  -> Az LSB
//Tel_data[10] -> Az MSB
//Tel_data[11] -> Gx LSB
//Tel_data[12] -> Gx MSB
//Tel_data[13] -> Gy LSB
//Tel_data[14] -> Gy MSB
//Tel_data[15] -> Gz LSB
//Tel_data[16] -> Gz MSB
//Tel_data[17] -> Flags 0b0000(dif)(bse)(bppc)(apps)
//Tel_data[18] -> motor1 temp
//Tel_data[19] -> MSB motor1 rpm
//Tel_data[20] -> LSB motor1 rpm
//Tel_data[21] -> motor2 temp
//Tel_data[22] -> MSB motor2 rpm
//Tel_data[23] -> LSB motor2 rpm
//Tel_data[24] -> Inversor errors
//Tel_data[25] -> MSB pack current
//Tel_data[26] -> LSB pack current
//Tel_data[27] -> MSB pack voltage
//Tel_data[28] -> LSB pack voltage
//Tel_data[29] -> state of charge
//Tel_data[30] -> high pack temp
//Tel_data[31] -> low pack temp


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Setup(void);
void Timer_Setup(void);
void Time_Analisys (void);
void MPU6050_init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void Differencial(void);
void PWM ( uint16_t left , uint16_t right );
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void Front_read(void);
double Saturate(double Min, double Value , double Max);
void Can_Rx_Wait(void);
void Set_MPU6050_Up( uint8_t Data, uint8_t reg );
double Max ( double value );
void Security_Elements(void);
void Start_Protocol(void);
void Brake_Process (void);
void Inv_Read(uint8_t inv_id);
void Can_Send_Retrans(uint8_t* buffer);
void Delay (void);
void Tx_Send ( uint32_t Id , uint32_t Dlc , int can_request );
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
  MX_WWDG_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	CAN_Setup();
	Timer_Setup();
	MPU6050_init();
	HAL_ADC_Start_DMA( &hadc1 , adc_buffer , 2 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		T1 = HAL_GetTick();
		
		//Can_Rx_Wait();
		
		//Front_read();
		
		MPU6050_Read_Accel();
		
		Security_Elements();
		
		while( flag_dif == 0 ){
			//Differencial();
		}
		
		
		T2 = HAL_GetTick();
		
		Time_Analisys ();
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	
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
	
	if ( T  < min ){
		min = T;
	}else{
		max = T;
	}
	
	Acc += T;  
	Average = (double) Acc/counter;
	
	//Tempo
	Tel_data[2] = (uint8_t)(counter&0xFF);
	Tel_data[3] = (uint8_t)((counter&0xFF00)>>8);
	Tel_data[4] = (uint8_t) T;

}

void MPU6050_init(void){
	uint8_t check;

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	
	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Set_MPU6050_Up ( 0x00 , 0x6B );
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Set_MPU6050_Up ( 0x07 , 0x19 );
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Set_MPU6050_Up ( 0x00 , 0x1C );
		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Set_MPU6050_Up ( 0x00 , 0x1B );
	}
		
}

void Set_MPU6050_Up( uint8_t Data, uint8_t reg ){
	
		uint8_t Config[2];
	
		Config[0] = Data;
		Config[1] = reg;
	
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, Config[1] , 1, &Config[0], 1, 1000);
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	
	if ( flag_accel == 0 ){
		
		for ( int i = 0 ; i < 50 ; i++ ){
			
			HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100);
			
			Ax = ((int16_t)(Rec_Data[0] << 8 | Rec_Data [1]))/16384.0;
			Ay = ((int16_t)(Rec_Data[2] << 8 | Rec_Data [3]))/16384.0;
			Az = ((int16_t)(Rec_Data[4] << 8 | Rec_Data [5]))/16384.0;
			
			Accx = Accx + Ax;
			Accy = Accy + Ay;
			Accz = Accz + Az;
			
			if( i == 0){
				Inf_x = Ax;
				Inf_y = Ay;
				Inf_z = Az;
			}
					//Determinação da faixa de janelamento
			if ( Ax > Sup_x ){
				Sup_x = Ax;
			}else if ( Ax < Inf_x){
				Inf_x = Ax;
			}
			
			if ( Ay > Sup_y ){
				Sup_y = Ay;
			}else if ( Ay < Inf_y){
				Inf_y = Ay;
			}
			
			if ( Az > Sup_z ){
				Sup_z = Az;
			}else if ( Az < Inf_z){
				Inf_z = Az;
			}
			
		}
		
		//Determinação do offset
		Offx = Accx/50;
		Offy = Accy/50;
		Offz = Accz/50;
		
		Accx = 0;
		Accy = 0;
		Accz = 0;

		flag_accel = 1;		
	}else{
		
		//Aplicação do filtro de média móvel
		
		for (int i = 0 ; i < 10 ; i++ ){
				
				HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, &Rec_Data[0], 2, 100);
			
				Ax = ((int16_t)(Rec_Data[0] << 8 | Rec_Data [1]))/16384.0;
			
				if((Ax < Sup_x) && (Ax > Inf_x)) Ax = 0; // Aplicação do janelamento
			
				Accx = Accx + Ax;
				
			}
			
			Ax = (Accx/10) - Offx;
	
	
			
		for (int i = 0 ; i < 10 ; i++ ){
				
				HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_YOUT_H_REG, 1, &Rec_Data[2], 2, 100);
			
				Ay = ((int16_t)(Rec_Data[2] << 8 | Rec_Data [3]))/16384.0;
			
				if((Ay < Sup_y) && (Ay > Inf_y)) Ay = 0;
			
				Accy = Accy + Ay;
				
			}
			
			Ay = (Accy/10) - Offy;
			
		for (int i = 0 ; i < 10 ; i++ ){
				
			HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_ZOUT_H_REG, 1, &Rec_Data[4], 2, 100);
			
			Az = ((int16_t)(Rec_Data[4] << 8 | Rec_Data [5]))/16384.0;
			
			if((Az < Sup_z) && (Az > Inf_z)) Az = 0;
			
			Accz = Accz + Az;
				
		}
			
		Az = (Accz/10) - Offz;
			
	}	
	
	//Acelerometro
	Tel_data[5] = (uint8_t)(Ax*0x00FF);
	Tel_data[6] = (uint8_t)(Ax*0xFF00) >> 8;
	Tel_data[7] = (uint8_t)(Ay*0x00FF);
	Tel_data[8] = (uint8_t)(Ay*0xFF00) >> 8;
	Tel_data[9] = (uint8_t)(Az*0x00FF);
	Tel_data[10] = (uint8_t)(Az*0xFF00) >> 8;	
	
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Gyro[6];
	
	if ( flag_gyro == 0 ){
		
		for ( int i = 0 ; i < 50 ; i++ ){
			
			HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Gyro, 6, 100);
			
			Gx = ((int16_t)(Rec_Gyro[0] << 8 | Rec_Gyro [1]))/131.0;
			Gy = ((int16_t)(Rec_Gyro[2] << 8 | Rec_Gyro [3]))/131.0;
			Gz = ((int16_t)(Rec_Gyro[4] << 8 | Rec_Gyro [5]))/131.0;
			
			Gccx = Gccx + Gx;
			Gccy = Gccy + Gy;
			Gccz = Gccz + Gz;
			
			if( i == 0){
				Inf_gx = Gx;
				Inf_gy = Gy;
				Inf_gz = Gz;
			}
			
			//Determinação da faixa de janelamento
			if ( Gx > Sup_gx ){
				Sup_gx = Gx;
			}else if ( Gx < Inf_gx){
				Inf_gx = Gx;
			}
			
			if ( Gy > Sup_gy ){
				Sup_gy = Gy;
			}else if ( Gy < Inf_gy){
				Inf_gy = Gy;
			}
			
			if ( Gz > Sup_gz ){
				Sup_gz = Gz;
			}else if ( Gz < Inf_gz){
				Inf_gz = Gz;
			}
			
		}
		
		//Determinação do offset
		Offgx = Gccx/50;
		Offgy = Gccy/50;
		Offgz = Gccz/50;
		
		Gccx = 0;
		Gccy = 0;
		Gccz = 0;

		flag_gyro = 1;		
	}else{
		
		//Aplicação do filtro de média móvel
		
			for (int i = 0 ; i < 10 ; i++ ){
				
				HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, &Rec_Gyro[0], 2, 100);
			
				Gx = ((int16_t)(Rec_Gyro[0] << 8 | Rec_Gyro [1]))/131.0;
				
				if ( (Gx < Sup_gx) && (Gx > Inf_gx)) Gx = 0; // Aplicação do janelamento
			
				Gccx = Gccx + Gx;
				
			}
			
			Gx = (Gccx/10) - Offgx;
			
			for (int i = 0 ; i < 10 ; i++ ){
				
				HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_YOUT_H_REG, 1, &Rec_Gyro[2], 2, 100);
			
				Gy = ((int16_t)(Rec_Gyro[2] << 8 | Rec_Gyro [3]))/131.0;
				
				if ( (Gy < Sup_gy) && (Gy > Inf_gy)) Gy = 0;
			
				Gccy = Gccy + Gy;
				
			}
			
			Gy = (Gccy/10) - Offgy;
			
			for (int i = 0 ; i < 10 ; i++ ){
				
				HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, &Rec_Gyro[4], 2, 100);
			
				Gz = ((int16_t)(Rec_Gyro[4] << 8 | Rec_Gyro[5]))/131.0;
				
				if ( (Gz < Sup_gz) && (Gz > Inf_gz)) Gz = 0;
			
				Gccz = Gccz + Gz;
				
			}
			
			Gz = (Gccz/10) - Offgz;
		
	}
	
	//Giroscopio
	Tel_data[11] = (uint8_t)(Gx*0x00FF);
	Tel_data[12] = (uint8_t)(Gx*0xFF00) >> 8;
	Tel_data[13] = (uint8_t)(Gy*0x00FF);
	Tel_data[14] = (uint8_t)(Gy*0xFF00) >> 8;
	Tel_data[15] = (uint8_t)(Gz*0x00FF);
	Tel_data[16] = (uint8_t)(Gz*0xFF00) >> 8;	
	
}

void Differencial(void){	
			
			/*K = (D*tan((steer*24.5)/(180/pi))) / (2*L);
			Dif_lef = (397*Throtle + 22)*(1 - K);
			Dif_rgt = (  397*Throtle + 22)*(1 + K);
			Dif_lef = Saturate(22, Dif_lef, 419);
			Dif_rgt = Saturate(22, Dif_rgt, 419);*/
				
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint16_t) dif_lef);
			//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,(uint16_t) dif_rgt);
	
			PWM( (uint16_t) dif_lef , (uint16_t) dif_rgt );
			
}

void PWM ( uint16_t left , uint16_t right ){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		
	if (hadc -> Instance == ADC1){
		for ( int i = 0 ; i < 2 ; i++ ){	
			if(adc_buffer_count < 20){ //faz a média dos 20 últimos valores coletados e envia por CAN
				adc_buffer_count++;
				Acc_adc[i] +=adc_buffer[i];
			}else{
				value[i] = Acc_adc[i]/20;
				Acc_adc[i] = 0;
				adc_buffer_count = 0;
		
				Brake0 = value[0];
				Brake1 = value[1];
				Brake_Process();
			}
		}
	}
}

void Brake_Process(void){

	Brake = (double)(Brake0 - Brake0_min)/(Brake0_max - Brake0_min);
	Brake = Saturate(0, Brake, 1);
	Tel_data[4] = (Brake*255);

}

void Front_read(){
	uint16_t Frontal_Id = 0x410;
	
	Tx_Send( Frontal_Id , 1 , True );

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

	Tel_data[0] = Throtle*255;
	Tel_data[1] = (Steer * 127) + 127;
	
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Tel_data[0], &TxMailbox);
	/*----------------------------------------------*/
}

void Tel_Send(void){

	int i;
	
	for ( i = 0 ; i < 4 ; i++ ){
		Tx_Send ( 0x610 + i , 8 , False );
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t*)&Tel_data[8*i], &TxMailbox);
	}
	
	flag_tel = 1;

}

void Display_Send (void){

		Tx_Send( 0x510 , 8 , False );
		HAL_CAN_AddTxMessage(&hcan, &TxHeader , &Tel_data[0] , &TxMailbox);

	//fazer o laço for
	
		flag_display = 1;
	
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


void Bms_Read(){
	uint16_t bms_pid[7] = {0xF00B, 0xF00C, 0xF014, 0xF00F, 0xF028, 0xF02A, 0xF032};
	uint8_t buffer[8];
	int i;
	
	TxHeader.StdId = 0x7E3;
	TxHeader.DLC = 8;
	buffer[0] = 0x04;
	buffer[1] = 0x22;
	for(i=0; i<7; i++){
		buffer[2] = (bms_pid[i] & 0xFF00) >> 8;
		buffer[3] = bms_pid[i] & 0x00FF;
		Can_Send_Retrans(buffer);
	}
}

void Can_Rx_Wait(){
	while(flag_can);
	flag_can = 1;
}

void Security_Elements(void){
	
	int count_error = 0, flag_error = 0;
	
	if ( flag_dif == 0 ){
		//Verificação do APPS: Se o pedal de aceleração for desconectado ou curto-circuitado, então o diferencial não funcionará
		if((Throtle0 < Throtle0_short) || (Throtle0 > Throtle0_open) || (Throtle1 < Throtle1_short) || (Throtle1 > Throtle1_open)){
			flag_apps = 1;
		}	
		//Verificação do BSE: Se o transdutor de pressão de freio for desconectado ou curto-circuitado, então o diferencial não funcionará
		if((Brake0 < Brake0_short) || (Brake0 > Brake0_open) || (Brake1 < Brake1_short) || (Brake1 > Brake0_open)){
			flag_bse = 1;
		}
		//Condição para desligamento dos motores
		if ( ( flag_apps == 1 ) || ( flag_bse == 1 ) ){
			flag_dif = 1;
			PWM( 18 , 18 );
			count_error++;
		}

		//Verificação do BPPC: Deve desligar os motores caso os freios sejam acionados enquanto o pedal de aceleração se encontra com uma excursão maior que 25%
		if ((Throtle > 0.25)&&(Brake > 0.1)){
			flag_dif = 1;
			flag_bppc = 1;
			PWM( 18 , 18 );
		}
	}	else{
		//verificar a condição de 100ms de espera
		if(count_error > 0){
			count_error++;
		
			if(count_error > 10){
				//Verificação do APPS: Se o pedal de aceleração for desconectado ou curto-circuitado, então o diferencial não funcionará
				if((Throtle0 < Throtle0_short) || (Throtle0 > Throtle0_open) || (Throtle1 < Throtle1_short) || (Throtle1 > Throtle1_open)){
					flag_error = 1;
				}	
				//Verificação do BSE: Se o transdutor de pressão de freio for desconectado ou curto-circuitado, então o diferencial não funcionará
				if((Brake0 < Brake0_short) || (Brake0 > Brake0_open) || (Brake1 < Brake1_short) || (Brake1 > Brake0_open)){
					flag_error = 1;
				}
			
				if(flag_error == 0){
					flag_dif = 0;
					count_error = 0;
				}
			}
		}else{
			if(Throtle < 0.05){
				flag_dif = 0;
				flag_bppc = 0;
		}
	}
	}
	
	//Flags
	Tel_data[17] = ((flag_apps&0x01)|((flag_bppc<<1)&0x02)|((flag_bse<<2)&0x04)|((flag_dif<<3)&0x08));
}

void Break_Light(void){
	if(Brake > 0.1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	else{
		if(Brake < 0.05){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
	}
}

void Inv_Read(uint8_t inv_id){
	uint8_t inv_com1[1] = {0x33};
	uint8_t inv_com2[1] = {0x37};
	
	Tx_Send ( inv_id , 1 , 0 );
	
	TxHeader.StdId = inv_id;
	TxHeader.DLC = 1;
	Can_Send_Retrans(inv_com1);
	Can_Send_Retrans(inv_com2);
}

void Can_Send_Retrans(uint8_t* buffer){
	uint16_t cnt0;
	
	while(1){
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, &buffer[0], &TxMailbox);
		cnt0 = __HAL_TIM_GET_COUNTER(&htim4);
		while(((__HAL_TIM_GET_COUNTER(&htim4) - cnt0) < 400) && flag_can == 1);
		if(flag_can == 0){
			flag_can = 1;
			break;
		}
	}
}

void Delay (void){
	for(int i=0; i<200; i++){
		while(flag_tim);
		flag_tim = 1;
	}
}

void Start_Protocol(void){
	uint8_t Button;
	
	Delay();
	
	do{
		Inv_Read(0x06B);
		Inv_Read(0x06C);
	}while((inv[0] | inv[1] | inv[2] | inv[3]) != 0x00);
	
	do{
		Front_read();
		Brake_Process();
		Button = Front_Data[6];
	}while(( Button != 1  )||( Brake < 0.8 ));
		
	HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port,Buzzer_Pin_Pin,GPIO_PIN_RESET);		// definidos no main.h		
	Delay ();
	HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port,Buzzer_Pin_Pin,GPIO_PIN_SET);		
	
	Tel_data[24] = (inv[0]&0x01)|((inv[1]<<1)&0x02)|((inv[2]<<2)&0x04)|((inv[3]<<2)&0x08);
			
}

void Tx_Send ( uint32_t Id , uint32_t Dlc , int can_request ){
	
	if ( can_request == False ){
		TxHeader.StdId = Id;
		TxHeader.DLC = Dlc;
	}else{
		TxHeader.StdId = Id;
		TxHeader.DLC = Dlc;
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, &dummy, &TxMailbox);
	}
	

}

/* USER CODE END 4 */

/**
  * @brief  This function is execute d in case of error occurrence.
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
