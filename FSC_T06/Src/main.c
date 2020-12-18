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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_FilterTypeDef set_Filter_up;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag_can = 1;
uint8_t Can_Data_Send[9];
int i;
uint32_t data[3], value[3], adc_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Can_Rx_Wait(void);
void CAN_Setup(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void CAN_Send_ADC(void);
void CAN_Send_GPIO(void);
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	CAN_Setup();
	HAL_ADC_Start_DMA( &hadc1 , adc_buffer , 3 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//Can_Rx_Wait();
		HAL_ADC_ConvCpltCallback(&hadc1);	
		CAN_Send_ADC();
		CAN_Send_GPIO();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CAN_Setup(void){
//---------Configurações de filtro e máscara---------

	set_Filter_up.FilterIdHigh = 0x400<<5;  
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

void Can_Rx_Wait(){
	while(flag_can);
	flag_can = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint32_t Acc = 0;
	if (hadc -> Instance == ADC1){
		for (  i = 0 ; i < 3 ; i++ ){
			for( int j = 0 ; j < 20 ; j++ ){
				Acc += adc_buffer[i];
			}
			value[i] = Acc/20;
			Acc = 0;
		}
	}
	
		Can_Data_Send[0] = (0x000000FF & value[0]);
		Can_Data_Send[1] = (0x0000FF00 & value[0]) >> 8;
		
		Can_Data_Send[2] = (0x000000FF & value[1]);
		Can_Data_Send[3] = (0x0000FF00 & value[1]) >> 8;
		
		Can_Data_Send[4] = (0x000000FF & value[2]);
		Can_Data_Send[5] = (0x0000FF00 & value[2]) >> 8;
		
}

void CAN_Send_ADC(){
	TxHeader.DLC = 6;
	TxHeader.StdId = 0x410;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Can_Data_Send[0], &TxMailbox);
}

void CAN_Send_GPIO(){
	TxHeader.DLC = 3;
	TxHeader.StdId = 0x420;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Can_Data_Send[6], &TxMailbox);
	
	Can_Data_Send[6] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	Can_Data_Send[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	Can_Data_Send[8] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
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
