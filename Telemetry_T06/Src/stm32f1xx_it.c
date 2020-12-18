/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint8_t Tel_data[32];
extern CAN_RxHeaderTypeDef RxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

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

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	
	volatile uint8_t CAN_Data_Receiving[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t *)&CAN_Data_Receiving[0]);

	
	if (RxHeader.StdId == 0x610){
		
		Tel_data[0] = CAN_Data_Receiving[0];
		Tel_data[1] = CAN_Data_Receiving[1];
		Tel_data[2] = CAN_Data_Receiving[2];
		Tel_data[3] = CAN_Data_Receiving[3];
		Tel_data[4] = CAN_Data_Receiving[4];
		Tel_data[5] = CAN_Data_Receiving[5];
		Tel_data[6] = CAN_Data_Receiving[6];
		Tel_data[7] = CAN_Data_Receiving[7];
	
	}
	
	if (RxHeader.StdId == 0x611){
		
		Tel_data[8] = CAN_Data_Receiving[0];
		Tel_data[9] = CAN_Data_Receiving[1];
		Tel_data[10] = CAN_Data_Receiving[2];
		Tel_data[11] = CAN_Data_Receiving[3];
		Tel_data[12] = CAN_Data_Receiving[4];
		Tel_data[13] = CAN_Data_Receiving[5];
		Tel_data[14] = CAN_Data_Receiving[6];
		Tel_data[15] = CAN_Data_Receiving[7];
	
	}
	
	if (RxHeader.StdId == 0x612){
		
		Tel_data[16] = CAN_Data_Receiving[0];
		Tel_data[17] = CAN_Data_Receiving[1];
		Tel_data[18] = CAN_Data_Receiving[2];
		Tel_data[19] = CAN_Data_Receiving[3];
		Tel_data[20] = CAN_Data_Receiving[4];
		Tel_data[21] = CAN_Data_Receiving[5];
		Tel_data[22] = CAN_Data_Receiving[6];
		Tel_data[23] = CAN_Data_Receiving[7];
	
	}
	
	if (RxHeader.StdId == 0x613){
		
		Tel_data[24] = CAN_Data_Receiving[0];
		Tel_data[25] = CAN_Data_Receiving[1];
		Tel_data[26] = CAN_Data_Receiving[2];
		Tel_data[27] = CAN_Data_Receiving[3];
		Tel_data[28] = CAN_Data_Receiving[4];
		Tel_data[29] = CAN_Data_Receiving[5];
		Tel_data[30] = CAN_Data_Receiving[6];
		Tel_data[31] = CAN_Data_Receiving[7];
	
	}


}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
