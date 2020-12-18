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
#include "can.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_stm32.h"
#include "ff.h"	
#include "ffconf.h"	
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef set_Filter_up;
CAN_TxHeaderTypeDef TxHeader;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char num[]="0";

FATFS mynewdiskFatFs; 
FIL MyFile; 
char mynewdiskPath[4];
uint8_t value;
uint16_t c = 990;

uint8_t Hour, min, sec;
uint8_t date,week_day,month,year;

uint32_t TxMailbox;

volatile uint8_t flag_can = 1;

/*-------Dados telemetria----------*/

volatile uint8_t Tel_data[38];

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
//Tel_data[31] -> Hora
//Tel_data[32] -> Minutos
//Tel_data[33] -> Segundos
//Tel_data[34] -> Ano
//Tel_data[35] -> Mês
//Tel_data[36] -> Dia


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Sd_Write(void);
void Sd_Save(void);
void Sd_Config(void);
void Can_Rx_Wait(void);
void read_RTC (void);
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
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	Sd_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		Sd_Write();
		if(c == 1000){
			Sd_Save();
			c = 1;
		}
		c++;
		
		
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CAN_Setup(void){
//---------Configurações de filtro e máscara---------

	set_Filter_up.FilterIdHigh = 0x600<<5;  
	set_Filter_up.FilterIdLow = 0; 
	set_Filter_up.FilterMaskIdHigh = 0x700<<5; 					
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

void Sd_Config(void){
	f_mount(&mynewdiskFatFs, (TCHAR const*)mynewdiskPath, 0);
	f_open(&MyFile, "T05.TXT", FA_OPEN_APPEND | FA_WRITE);
}

void Sd_Write(void){
	char text[4], textf[5];
	uint32_t bytes;
	
	for(int i=0; i<37; i++){
		sprintf(text,"%03d", Tel_data[i] );       
		text[3] = ',';
		f_write(&MyFile, text, sizeof(text), (void *)&bytes);
	}
	
	sprintf(textf,"%03d\r\n", Tel_data[37]);
	f_write(&MyFile, textf, sizeof(textf), (void *)&bytes);
	
}

void Sd_Save(void){
	f_close(&MyFile);
	f_open(&MyFile, "T05.TXT", FA_OPEN_APPEND | FA_WRITE);
}

void read_RTC (void){
	
	if (HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD) == HAL_OK){
		Hour = sTime.Hours;
		min = sTime.Minutes;
		sec = sTime.Seconds;
	}else{
		Error_Handler();
	}
	
	if( Hour == 0 ){	
		if (HAL_RTC_GetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BCD) == HAL_OK){
			year = DateToUpdate.Year;
			month = DateToUpdate.Month;
			week_day = DateToUpdate.WeekDay;
			date = DateToUpdate.Date;
		}else{
			Error_Handler();
		}	
	}
	
	Tel_data[32] = Hour;
	Tel_data[33] = min;
	Tel_data[34] = sec;
	Tel_data[35] = year;
	Tel_data[36] = month;
	Tel_data[37] = date;
}

void Can_Receive(void){
	uint8_t dummy = 0x00;
	
	TxHeader.DLC = 1;
	TxHeader.StdId = 0x310;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, &dummy, &TxMailbox);
	
	Can_Rx_Wait();
	Can_Rx_Wait();
	Can_Rx_Wait();
	Can_Rx_Wait();
}

void Can_Rx_Wait(void){
	while(flag_can);
	flag_can = 0;
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
