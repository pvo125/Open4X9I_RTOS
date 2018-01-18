/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "mytask.h"
#include "PS2_Keyboard.h"
#include "CAN.h"

extern char CommandMsgBuffer[];
extern osMessageQId QueueCANRecieveHandle;
extern volatile uint8_t scancode;
extern volatile PS2_KBD_VAR_t		PS2_KBD_VAR;
extern const char * CANNodeBuffer[];
extern uint8_t NetIndexBuff[];


volatile uint8_t message_mode=0;



/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA2D_HandleTypeDef hdma2d;
extern LTDC_HandleTypeDef hltdc;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim14;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RTC wake-up interrupt through EXTI line 22.
*/
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */
	
  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */
	osSemaphoreRelease(xRTCSemaphoreHandle);
  /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	osSemaphoreRelease(xLED1SemaphoreHandle);
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
	osSemaphoreRelease(xLED2SemaphoreHandle);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	osSemaphoreRelease(xLED3SemaphoreHandle);
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint8_t i;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if(message_mode)
	{
		if(USART1->DR=='\r')
			osSemaphoreRelease(xDMA2Stream2SemaphoreHandle);
	}
	else
	{	
		if(USART1->DR=='\r')
		{
		i=MSGBUFF_SIZE-DMA2_Stream2->NDTR;
		CommandMsgBuffer[i-1]=0;
		osSemaphoreRelease(xInputMsgSemaphoreHandle);
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	osSemaphoreRelease(xLED4SemaphoreHandle);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	osSemaphoreRelease(xTIM6SemaphoreHandle);
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
	osSemaphoreRelease(xTIM7SemaphoreHandle);
  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
 //__HAL_DMA_CLEAR_FLAG(&hdma2,DMA_FLAG_TCIF3_7);
 	//osSemaphoreRelease(xDMA2Stream2SemaphoreHandle);
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	//osSemaphoreRelease(xDMA2Stream7SemaphoreHandle);
  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
* @brief This function handles LTDC global interrupt.
*/
void LTDC_IRQHandler(void)
{
  /* USER CODE BEGIN LTDC_IRQn 0 */

  /* USER CODE END LTDC_IRQn 0 */
  HAL_LTDC_IRQHandler(&hltdc);
  /* USER CODE BEGIN LTDC_IRQn 1 */

  /* USER CODE END LTDC_IRQn 1 */
}

/**
* @brief This function handles DMA2D global interrupt.
*/
void DMA2D_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2D_IRQn 0 */
	
  /* USER CODE END DMA2D_IRQn 0 */
  HAL_DMA2D_IRQHandler(&hdma2d);
  /* USER CODE BEGIN DMA2D_IRQn 1 */
	DMA2D->CR      = 0x00010000UL | (1 << 9);
  osSemaphoreRelease(xDMA2DSemaphoreHandle);
  /* USER CODE END DMA2D_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void TIM3_IRQHandler(void){
		
	if(TIMx->SR&TIM_SR_UIF)
		{
			TIMx->SR&=~TIM_SR_TIF;
			if(PS2_KBD_VAR.mode==RX_MODE)
			{
				PS2_KBD_CheckScancode(scancode);
				TIMx->SR&=~TIM_SR_UIF;
				osSemaphoreRelease(xPS2CharSemaphoreHandle);
			}
			else if (PS2_KBD_VAR.mode==TX_MODE)
				PS2_KBD_VAR.mode=RX_MODE;
		}
	else if(TIMx->SR&TIM_SR_TIF)
		{
			PS2_KBD_ISR();
			TIMx->SR&=~TIM_SR_TIF;
		}
}
/*
*/
void CAN2_TX_IRQHandler(void){

	
	CAN2->TSR |=CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2;
	osSemaphoreRelease(xCAN_TXSemaphoreHandle);
}

/*
*/
void CAN2_RX0_IRQHandler(void){
	CANRX_TypeDef CAN_Data_RX;
	portBASE_TYPE xHigherPriorityTaskWoken;
	
			if((CAN2->sFIFOMailBox[0].RIR&CAN_RI0R_RTR)!=CAN_RI0R_RTR)
			{
				*(uint32_t*)CAN_Data_RX.Data=(uint32_t)CAN2->sFIFOMailBox[0].RDLR;
				
				*(uint32_t*)(CAN_Data_RX.Data+4)=(uint32_t)CAN2->sFIFOMailBox[0].RDHR;
				
				CAN_Data_RX.DLC=(uint8_t)0x0F & CAN2->sFIFOMailBox[0].RDTR;
			}
			CAN_Data_RX.ID=(uint16_t)0x7FF & (CAN2->sFIFOMailBox[0].RIR>>21);
			CAN_Data_RX.FMI=(uint8_t)0xFF & (CAN2->sFIFOMailBox[0].RDTR>>8);
			CAN_Data_RX.FIFO=0;
			CAN2->RF0R|=CAN_RF0R_RFOM0;		/*Release FIFO_0*/		
			
		xHigherPriorityTaskWoken=pdFALSE;	
		xQueueSendToBackFromISR(xQueueCANRX,&CAN_Data_RX,&xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken==pdTRUE)
			portYIELD();	
}

/*
*/
void CAN2_RX1_IRQHandler(void){
	CANRX_TypeDef CAN_Data_RX;
	portBASE_TYPE xHigherPriorityTaskWoken;
	
	if((CAN2->sFIFOMailBox[1].RIR&CAN_RI0R_RTR)!=CAN_RI0R_RTR)
			{
			*(uint32_t*)CAN_Data_RX.Data=(uint32_t)CAN2->sFIFOMailBox[1].RDLR;
			*(uint32_t*)(CAN_Data_RX.Data+4)=(uint32_t)CAN2->sFIFOMailBox[1].RDHR;
				CAN_Data_RX.DLC=(uint8_t)0x0F & CAN2->sFIFOMailBox[1].RDTR;
			}
		CAN_Data_RX.ID=(uint16_t)0x7FF & (CAN2->sFIFOMailBox[1].RIR>>21);
		CAN_Data_RX.FMI=(uint8_t)0xFF & (CAN2->sFIFOMailBox[1].RDTR>>8);
		CAN_Data_RX.FIFO=1;
		CAN2->RF1R|=CAN_RF1R_RFOM1;		/*Release FIFO_1*/		
		
		xHigherPriorityTaskWoken=pdFALSE;	
		xQueueSendToBackFromISR(xQueueCANRX,&CAN_Data_RX,&xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken==pdTRUE)
			portYIELD();
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
