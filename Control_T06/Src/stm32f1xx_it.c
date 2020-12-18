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
extern CAN_RxHeaderTypeDef RxHeader;
extern TIM_HandleTypeDef htim4;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t flag_can;
extern uint8_t Front_Data[9];
extern uint8_t flag_tim;
extern uint8_t inv[4];
extern uint8_t bms[2];
extern uint8_t Tel_data[32];
extern int flag_tel;
extern uint8_t flag_display;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern WWDG_HandleTypeDef hwwdg;
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
  * @brief This function handles Window watchdog interrupt.
  */
void WWDG_IRQHandler(void)
{
  /* USER CODE BEGIN WWDG_IRQn 0 */

  /* USER CODE END WWDG_IRQn 0 */
  HAL_WWDG_IRQHandler(&hwwdg);
  /* USER CODE BEGIN WWDG_IRQn 1 */

  /* USER CODE END WWDG_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

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
	
		if (RxHeader.StdId == 0x410){	
			Front_Data[0] = CAN_Data_Receiving[0]; 
			Front_Data[1] = CAN_Data_Receiving[1];
			Front_Data[2] = CAN_Data_Receiving[2];
			Front_Data[3] = CAN_Data_Receiving[3];
			Front_Data[4] = CAN_Data_Receiving[4];
			Front_Data[5] = CAN_Data_Receiving[5];
	  }
		
		if (RxHeader.StdId == 0x420){
			Front_Data[6] = CAN_Data_Receiving[0];
			Front_Data[7] = CAN_Data_Receiving[1];
			Front_Data[8] = CAN_Data_Receiving[2];
		}
		
		if (RxHeader.StdId == 0x073){
			if(RxHeader.DLC == 6){
				Tel_data[18] = CAN_Data_Receiving[2];
			}
			if(RxHeader.DLC == 5){
				Tel_data[19] = CAN_Data_Receiving[0];
				Tel_data[20] = CAN_Data_Receiving[1];
				inv[0] = CAN_Data_Receiving[3];
				inv[1] = CAN_Data_Receiving[4];
			}
		}
	
		if (RxHeader.StdId == 0x074){
			if(RxHeader.DLC == 6){
				Tel_data[21] = CAN_Data_Receiving[2];
			}
			if(RxHeader.DLC == 5){
				Tel_data[22] = CAN_Data_Receiving[0];
				Tel_data[23] = CAN_Data_Receiving[1];
				inv[2] = CAN_Data_Receiving[3];
				inv[3] = CAN_Data_Receiving[4];
			}		
		}
		if(RxHeader.StdId == 0x7EB) {
			if(CAN_Data_Receiving[3] == 0x0B) {			
				bms[0] = CAN_Data_Receiving[4];
				bms[1] = CAN_Data_Receiving[5];
		}
			
		if(CAN_Data_Receiving[3] == 0x0C){
				Tel_data[25] = CAN_Data_Receiving[4];
				Tel_data[26] = CAN_Data_Receiving[5];
		}
		
		if(CAN_Data_Receiving[3] == 0x14){
				Tel_data[27] = CAN_Data_Receiving[4];
				Tel_data[28] = CAN_Data_Receiving[5];
		}
		
		if(CAN_Data_Receiving[3] == 0x0F){
				Tel_data[29] = CAN_Data_Receiving[4];
		}
		
		if(CAN_Data_Receiving[3] == 0x28){
				Tel_data[30] = CAN_Data_Receiving[4];
		}
		
		if(CAN_Data_Receiving[3] == 0x2A){
				Tel_data[31] = CAN_Data_Receiving[4];
		}
	}
		
	if (RxHeader.StdId == 0x310 ){
		flag_tel = 0;
	}
	
	if (RxHeader.StdId == 0x320){
		flag_display = 0;
	}

	flag_can = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4){
		flag_tim = 0;
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
