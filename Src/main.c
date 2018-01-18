/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "mytask.h"
#include "LCDdriver.h"
#include "PS2_Keyboard.h"
#include "CAN.h"

#define REFRESH_COUNT           ((uint32_t)1386)   /* SDRAM refresh counter */
#define SDRAM_TIMEOUT           ((uint32_t)0xFFFF)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

SDRAM_HandleTypeDef hsdram2;

osThreadId defaultTaskHandle;
osThreadId LCDPrintCurrentHandle;
osThreadId PrintKernelLoadHandle;
osThreadId PrintRTCHandle;
osThreadId LED1BlinkHandle;
osThreadId LED2BlinkHandle;
osThreadId LED3BlinkHandle;
osThreadId LED4BlinkHandle;
osThreadId CommandMsgDecodHandle;
osThreadId LCDBlinkCursorHandle;
osThreadId PS2CharDecodeHandle;
osMessageQId QueuePrintLCDHandle;
osMessageQId QueueCANRecieveHandle;
osMessageQId QueueCANTransmiteHandle;
osMutexId xCANStructPointHandle;
osMutexId xDMA2Stream2Handle;
osSemaphoreId xRTCSemaphoreHandle;
osSemaphoreId xDMA2DSemaphoreHandle;
osSemaphoreId xBlinkCursorSemaphoreHandle;
osSemaphoreId xLED1SemaphoreHandle;
osSemaphoreId xLED2SemaphoreHandle;
osSemaphoreId xLED3SemaphoreHandle;
osSemaphoreId xLED4SemaphoreHandle;
osSemaphoreId xInputMsgSemaphoreHandle;
osSemaphoreId xCPULoadSemaphoreHandle;
osSemaphoreId xDMA2Stream2SemaphoreHandle;
osSemaphoreId xPS2CharSemaphoreHandle;
osSemaphoreId xTIM6SemaphoreHandle;
osSemaphoreId xTIM7SemaphoreHandle;
osSemaphoreId xDMA2Stream7SemaphoreHandle;
osSemaphoreId xCAN_TXSemaphoreHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

xQueueHandle xQueueCANTX,xQueueCANRX;
//sThreadId TaskHandle1,TaskHandle2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
extern void vLCDPrintCurrent(void const * argument);
extern void vPrintKernelLoad(void const * argument);
extern void vPrintRTC(void const * argument);
extern void vLED1Blink(void const * argument);
extern void vLED2Blink(void const * argument);
extern void vLED3Blink(void const * argument);
extern void vLED4Blink(void const * argument);
extern void vCommandMsgDecode(void const * argument);
extern void vLCDBlinkCursor(void const * argument);
extern void vPS2CharDecode(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void SDRAM_Initialization_sequence(uint32_t RefreshCount);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
		SDRAM_Initialization_sequence(REFRESH_COUNT);
		HAL_RTCEx_EnableBypassShadow(&hrtc);	
		
		
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of xCANStructPoint */
  osMutexDef(xCANStructPoint);
  xCANStructPointHandle = osMutexCreate(osMutex(xCANStructPoint));

  /* definition and creation of xDMA2Stream2 */
  osMutexDef(xDMA2Stream2);
  xDMA2Stream2Handle = osMutexCreate(osMutex(xDMA2Stream2));

  /* USER CODE BEGIN RTOS_MUTEX */
  	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of xRTCSemaphore */
  osSemaphoreDef(xRTCSemaphore);
  xRTCSemaphoreHandle = osSemaphoreCreate(osSemaphore(xRTCSemaphore), 1);

  /* definition and creation of xDMA2DSemaphore */
  osSemaphoreDef(xDMA2DSemaphore);
  xDMA2DSemaphoreHandle = osSemaphoreCreate(osSemaphore(xDMA2DSemaphore), 1);

  /* definition and creation of xBlinkCursorSemaphore */
  osSemaphoreDef(xBlinkCursorSemaphore);
  xBlinkCursorSemaphoreHandle = osSemaphoreCreate(osSemaphore(xBlinkCursorSemaphore), 1);

  /* definition and creation of xLED1Semaphore */
  osSemaphoreDef(xLED1Semaphore);
  xLED1SemaphoreHandle = osSemaphoreCreate(osSemaphore(xLED1Semaphore), 1);

  /* definition and creation of xLED2Semaphore */
  osSemaphoreDef(xLED2Semaphore);
  xLED2SemaphoreHandle = osSemaphoreCreate(osSemaphore(xLED2Semaphore), 1);

  /* definition and creation of xLED3Semaphore */
  osSemaphoreDef(xLED3Semaphore);
  xLED3SemaphoreHandle = osSemaphoreCreate(osSemaphore(xLED3Semaphore), 1);

  /* definition and creation of xLED4Semaphore */
  osSemaphoreDef(xLED4Semaphore);
  xLED4SemaphoreHandle = osSemaphoreCreate(osSemaphore(xLED4Semaphore), 1);

  /* definition and creation of xInputMsgSemaphore */
  osSemaphoreDef(xInputMsgSemaphore);
  xInputMsgSemaphoreHandle = osSemaphoreCreate(osSemaphore(xInputMsgSemaphore), 1);

  /* definition and creation of xCPULoadSemaphore */
  osSemaphoreDef(xCPULoadSemaphore);
  xCPULoadSemaphoreHandle = osSemaphoreCreate(osSemaphore(xCPULoadSemaphore), 1);

  /* definition and creation of xDMA2Stream2Semaphore */
  osSemaphoreDef(xDMA2Stream2Semaphore);
  xDMA2Stream2SemaphoreHandle = osSemaphoreCreate(osSemaphore(xDMA2Stream2Semaphore), 1);

  /* definition and creation of xPS2CharSemaphore */
  osSemaphoreDef(xPS2CharSemaphore);
  xPS2CharSemaphoreHandle = osSemaphoreCreate(osSemaphore(xPS2CharSemaphore), 1);

  /* definition and creation of xTIM6Semaphore */
  osSemaphoreDef(xTIM6Semaphore);
  xTIM6SemaphoreHandle = osSemaphoreCreate(osSemaphore(xTIM6Semaphore), 1);

  /* definition and creation of xTIM7Semaphore */
  osSemaphoreDef(xTIM7Semaphore);
  xTIM7SemaphoreHandle = osSemaphoreCreate(osSemaphore(xTIM7Semaphore), 1);

  /* definition and creation of xDMA2Stream7Semaphore */
  osSemaphoreDef(xDMA2Stream7Semaphore);
  xDMA2Stream7SemaphoreHandle = osSemaphoreCreate(osSemaphore(xDMA2Stream7Semaphore), 1);

  /* definition and creation of xCAN_TXSemaphore */
  osSemaphoreDef(xCAN_TXSemaphore);
  xCAN_TXSemaphoreHandle = osSemaphoreCreate(osSemaphore(xCAN_TXSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LCDPrintCurrent */
  osThreadDef(LCDPrintCurrent, vLCDPrintCurrent, osPriorityNormal, 0, 128);
  LCDPrintCurrentHandle = osThreadCreate(osThread(LCDPrintCurrent), NULL);

  /* definition and creation of PrintKernelLoad */
  osThreadDef(PrintKernelLoad, vPrintKernelLoad, osPriorityNormal, 0, 128);
  PrintKernelLoadHandle = osThreadCreate(osThread(PrintKernelLoad), NULL);

  /* definition and creation of PrintRTC */
  osThreadDef(PrintRTC, vPrintRTC, osPriorityNormal, 0, 128);
  PrintRTCHandle = osThreadCreate(osThread(PrintRTC), NULL);

  /* definition and creation of LED1Blink */
  osThreadDef(LED1Blink, vLED1Blink, osPriorityNormal, 0, 128);
  LED1BlinkHandle = osThreadCreate(osThread(LED1Blink), NULL);

  /* definition and creation of LED2Blink */
  osThreadDef(LED2Blink, vLED2Blink, osPriorityNormal, 0, 128);
  LED2BlinkHandle = osThreadCreate(osThread(LED2Blink), NULL);

  /* definition and creation of LED3Blink */
  osThreadDef(LED3Blink, vLED3Blink, osPriorityNormal, 0, 128);
  LED3BlinkHandle = osThreadCreate(osThread(LED3Blink), NULL);

  /* definition and creation of LED4Blink */
  osThreadDef(LED4Blink, vLED4Blink, osPriorityNormal, 0, 128);
  LED4BlinkHandle = osThreadCreate(osThread(LED4Blink), NULL);

  /* definition and creation of CommandMsgDecod */
  osThreadDef(CommandMsgDecod, vCommandMsgDecode, osPriorityNormal, 0, 128);
  CommandMsgDecodHandle = osThreadCreate(osThread(CommandMsgDecod), NULL);

  /* definition and creation of LCDBlinkCursor */
  osThreadDef(LCDBlinkCursor, vLCDBlinkCursor, osPriorityLow, 0, 128);
  LCDBlinkCursorHandle = osThreadCreate(osThread(LCDBlinkCursor), NULL);

  /* definition and creation of PS2CharDecode */
  osThreadDef(PS2CharDecode, vPS2CharDecode, osPriorityNormal, 0, 128);
  PS2CharDecodeHandle = osThreadCreate(osThread(PS2CharDecode), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
		
	osThreadSuspend (LCDPrintCurrentHandle);
	osThreadSuspend (PrintKernelLoadHandle);
	osThreadSuspend (PrintRTCHandle);
	osThreadSuspend (LED1BlinkHandle);
	osThreadSuspend (LED2BlinkHandle);
	osThreadSuspend (LED3BlinkHandle);
	osThreadSuspend (LED4BlinkHandle);
	osThreadSuspend(CommandMsgDecodHandle);
	osThreadSuspend(LCDBlinkCursorHandle);
	osThreadSuspend(PS2CharDecodeHandle);
	
	osSemaphoreWait(xLED1SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xLED2SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xLED3SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xLED4SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xDMA2Stream2SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xInputMsgSemaphoreHandle,osWaitForever);
	osSemaphoreWait(xPS2CharSemaphoreHandle,osWaitForever);
	osSemaphoreWait(xTIM6SemaphoreHandle,osWaitForever);
	osSemaphoreWait(xTIM7SemaphoreHandle,osWaitForever);
	
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	
	TIM6->DIER |= TIM_DIER_UIE; 				//разрешаем прерывание от таймера обновлени€ структуры GUI_PID_STATE
	TIM6->EGR = TIM_EGR_UG;							//генерируем "update event". ARR и PSC груз€тс€ из предварительного в теневой регистр. 
	TIM6->SR&=~TIM_SR_UIF; 							//—брасываем флаг UIF
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	
	TIM7->DIER |= TIM_DIER_UIE; 				//разрешаем прерывание от таймера обновлени€ структуры GUI_PID_STATE
	TIM7->EGR = TIM_EGR_UG;							//генерируем "update event". ARR и PSC груз€тс€ из предварительного в теневой регистр. 
	TIM7->SR&=~TIM_SR_UIF; 							//—брасываем флаг UIF
	NVIC_ClearPendingIRQ(TIM7_IRQn);
	
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of QueuePrintLCD */
  osMessageQDef(QueuePrintLCD, 16, sizeof(LCDPrintTypeDef));
  QueuePrintLCDHandle = osMessageCreate(osMessageQ(QueuePrintLCD), NULL);

  /* definition and creation of QueueCANRecieve */
  //osMessageQDef(QueueCANRecieve, 10, sizeof(CANRX_TypeDef));
 // QueueCANRecieveHandle = osMessageCreate(osMessageQ(QueueCANRecieve), NULL);

  /* definition and creation of QueueCANTransmite */
 // osMessageQDef(QueueCANTransmite, 16, sizeof(CANTX_TypeDef));
 // QueueCANTransmiteHandle = osMessageCreate(osMessageQ(QueueCANTransmite), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
		xQueueCANTX=xQueueCreate(4,sizeof(CANTX_TypeDef));
		xQueueCANRX=xQueueCreate(16,sizeof(CANRX_TypeDef));
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 288;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_16;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_A4;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 524;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 240;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 240;
  pLayerCfg.Backcolor.Blue = 150;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 100;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0xD00ff000;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 150;
  pLayerCfg1.Backcolor.Green = 100;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0x12;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x17;
  sDate.Year = 0x17;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the WakeUp 
    */
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 44999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 44999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 4499;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 200;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim13);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_2;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Led1_Pin|Led2_Pin|Led3_Pin|Led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led1_Pin Led2_Pin Led3_Pin Led4_Pin */
  GPIO_InitStruct.Pin = Led1_Pin|Led2_Pin|Led3_Pin|Led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Joystick_A_Pin Joystick_B_Pin Joystick_C_Pin Joystick_D_Pin */
  GPIO_InitStruct.Pin = Joystick_A_Pin|Joystick_B_Pin|Joystick_C_Pin|Joystick_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void SDRAM_Initialization_sequence(uint32_t RefreshCount)
{
	uint32_t tmp;
	
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
	#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
	#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
	#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
	#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
	#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
	#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
	#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
	#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
	#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
	#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
	
	FMC_SDRAM_CommandTypeDef Command;
	
		
	__IO uint32_t tmpmrd =0;
  
  /* Step 1:  Configure a clock configuration enable command */
  Command.CommandMode             = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram2, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */ 
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */ 
  Command.CommandMode             = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram2, &Command, SDRAM_TIMEOUT);  
  
  /* Step 4: Configure an Auto Refresh command */ 
  Command.CommandMode             = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 4;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram2, &Command, SDRAM_TIMEOUT);
  
  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
  Command.CommandMode             = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram2, &Command, SDRAM_TIMEOUT);
  
  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&hsdram2, RefreshCount); 

// Clear SDRAM 
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	for(tmpmrd=SDRAM_BASE;tmpmrd<(SDRAM_BASE+SDRAM_SIZE);tmpmrd+=4)
	*(uint32_t*)tmpmrd=0x0;
	
#if 0
// Initialization step 3	
		FMC_Bank5_6->SDCMR=FMC_SDCMR_MODE_0|								// 001 Clock Configuration Enable	
											 FMC_SDCMR_CTB2  |								// Command issued to SDRAM Bank 2		
											 FMC_SDCMR_NRFS_0;								// 2 Auto-refresh cycles
// Initialization step 4
		/* Delay 100 mS*/
		for(tmp = 0; tmp < 1000000; tmp++);
// Initialization step 5
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR=FMC_SDCMR_MODE_1|									// 010 PALL (УAll Bank PrechargeФ) command
										 FMC_SDCMR_CTB2	 |									// Command issued to SDRAM Bank 2
										 FMC_SDCMR_NRFS_0;									// 2 Auto-refresh cycles
// Initialization step 6
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR=  FMC_SDCMR_MODE_2|								// 011: Auto-refresh command
										   FMC_SDCMR_CTB2|									// Command issued to SDRAM Bank 2
	FMC_SDCMR_NRFS_0|FMC_SDCMR_NRFS_1|FMC_SDCMR_NRFS_2;		// Number of Auto-refresh 8 cycle (0111)
// Initialization step 7
 #define SDRAM_BURST_LENGTH_1                ((uint16_t)0x0000)
 #define SDRAM_BURST_LENGTH_2                ((uint16_t)0x0001)
 #define SDRAM_BURST_LENGTH_4                ((uint16_t)0x0002)
 #define SDRAM_BURST_LENGTH_8                ((uint16_t)0x0004)
 #define SDRAM_BURST_TYPE_SEQUENTIAL         ((uint16_t)0x0000)
 #define SDRAM_BURST_TYPE_INTERLEAVED        ((uint16_t)0x0008)
 #define SDRAM_CAS_LATENCY_2                 ((uint16_t)0x0020)
 #define SDRAM_CAS_LATENCY_3                 ((uint16_t)0x0030)
 #define SDRAM_OPERATING_MODE_STANDARD       ((uint16_t)0x0000)
 #define SDRAM_WRITEBURST_MODE_PROGRAMMED    ((uint16_t)0x0000)
 #define SDRAM_WRITEBURST_MODE_SINGLE        ((uint16_t)0x0200)
 #define FMC_SDCMR_MODE_3										 ((uint32_t)0x00000004)		
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR=FMC_SDCMR_MODE_3|									// Command mode 100: Load Mode Register 
											FMC_SDCMR_CTB2 |									//   Command issued to SDRAM Bank 2
											FMC_SDCMR_NRFS_0|							  	// 2 Auto-refresh cycles
											SDRAM_BURST_LENGTH_2<<9|
											SDRAM_BURST_TYPE_SEQUENTIAL<<9|
											SDRAM_CAS_LATENCY_3<<9|
											SDRAM_OPERATING_MODE_STANDARD<<9|
											SDRAM_WRITEBURST_MODE_SINGLE<<9;										
// Initialization step 8
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	FMC_Bank5_6->SDRTR|=(1386<<1);													// 64mS/4096=15.625uS
																													// 15.625*90MHz-20=1386
	// Clear SDRAM 
	while(FMC_Bank5_6->SDSR&FMC_SDSR_BUSY);
	for(tmp=SDRAM_BASE;tmp<(SDRAM_BASE+SDRAM_SIZE);tmp+=4)
	*(uint32_t*)tmp=0x0;
#endif

}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	
  /* Infinite loop */
  for(;;)
  {
   	LCD_PutString("EN",LCD_XSIZE-390,2,1,MAGENTA,0xFF);
		LCD_PutString("Load",LCD_XSIZE-357,2,1,GREEN,0xFF);
		LCD_PutString("Heap",LCD_XSIZE-100,26,1,GREEN,0xFF);
				
		
		LCD_ClearRec(0,0,LCD_XSIZE,2,LAYER_1,YELLOW,180);
		LCD_ClearRec(2,24,LCD_XSIZE-4,2,LAYER_1,YELLOW,180);
		LCD_ClearRec(2,240,LCD_XSIZE-4,2,LAYER_1,YELLOW,180);
		LCD_ClearRec(0,270,LCD_XSIZE,2,LAYER_1,YELLOW,180);
				
		LCD_ClearRec(LCD_XSIZE-395,2,2,22,LAYER_1,YELLOW,180);
		LCD_ClearRec(LCD_XSIZE-365,2,2,22,LAYER_1,YELLOW,180);
		LCD_ClearRec(LCD_XSIZE-200,2,2,22,LAYER_1,YELLOW,180);
		LCD_ClearRec(LCD_XSIZE-90,2,2,22,LAYER_1,YELLOW,180);
		
		LCD_ClearRec(0,0,2,LCD_YSIZE,LAYER_1,YELLOW,180);
		LCD_ClearRec(LCD_XSIZE-2,0,2,LCD_YSIZE,LAYER_1,YELLOW,180);
		
		
		
		//LCD_PutString("*****************************************************", LCD_XSIZE-479,228,1,GREEN,0xFF);
		PS2_KBD_LowLevelInit();
		if(PS2_KBD_Tx(0xFF)!=SUCCESS)
			PS2_KBD_LowLevelDeInit();
		
		bxCAN_Init();
		
		osDelay(1000);
				
		osThreadResume (LCDPrintCurrentHandle);
		osThreadResume (PrintKernelLoadHandle);
		osThreadResume (PrintRTCHandle);
		osThreadResume(LED1BlinkHandle);
		osThreadResume(LED2BlinkHandle);
		osThreadResume(LED3BlinkHandle);
		osThreadResume(LED4BlinkHandle);
		osThreadResume(CommandMsgDecodHandle);
		osThreadResume(LCDBlinkCursorHandle);
		osThreadResume(PS2CharDecodeHandle);
		osThreadTerminate (defaultTaskHandle);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */
		
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
