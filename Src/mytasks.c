#include "stm32f4xx.h"                  // Device header
#include "cmsis_os.h"
#include "timers.h"
#include "mytask.h"
#include "LCDdriver.h"
#include <string.h>
#include "PS2_Keyboard.h"
#include "CAN.h"



#define LCD_LAYER0_FRAME_BUFFER  ((uint32_t)0xD0000000)
#define countof(A) sizeof(A)/sizeof(A[0])
	


xTaskHandle CANNetID,CANTX,UART_Download,
Timer2000,Timer1000;

extern CANNode_TypeDef *pCANNode;
extern uint8_t NetIndexBuff[];

extern volatile uint8_t countnode;
volatile uint8_t last_countnode=0;

extern volatile uint8_t GlobalUpdateFlag;
extern volatile uint8_t indexFirmware;
extern volatile int32_t sizebin;
extern volatile uint32_t countbyte_firmware;

void vTimer2000ms(void * pvParameters);
void vTimer1000ms(void * pvParameters);

extern TIM_HandleTypeDef htim6,htim7;
extern RTC_HandleTypeDef hrtc;
extern LTDC_HandleTypeDef hltdc;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern volatile PS2_KBD_VAR_t		PS2_KBD_VAR;
extern const GUI_CHARINFO GUI_FontArialNarrow22_CharInfo[];

volatile uint16_t CursorPos=5;
volatile uint16_t cpuload;
volatile uint16_t current_line=218;


char CommandMsgBuffer[MSGBUFF_SIZE];
volatile ControlKeyTypeDef ControlKey;
const KeyTypeDef PS2Buff[]={{0x1C,'A','a','Ф','ф'},
																{0x32,'B','b','И','и'},
																{0x21,'C','c','С','с'},
																{0x23,'D','d','В','в'},
																{0x24,'E','e','У','у'},
																{0x2B,'F','f','А','а'},
																{0x34,'G','g','П','п'},
																{0x33,'H','h','Р','р'},
																{0x43,'I','i','Ш','ш'},
																{0x3B,'J','j','О','о'},
																
																{0x42,'K','k','Л','л'},
																{0x4B,'L','l','Д','д'},
																{0x3A,'M','m','Ь','ь'},
																{0x31,'N','n','Т','т'},
																{0x44,'O','o','Щ','щ'},
																{0x4D,'P','p','З','з'},
																{0x15,'Q','q','Й','й'},
																{0x2D,'R','r','К','к'},
																{0x1B,'S','s','Ы','ы'},
																{0x2C,'T','t','Е','е'},
																
																{0x3C,'U','u','Г','г'},
																{0x2A,'V','v','М','м'},
																{0x1D,'W','w','Ц','ц'},
																{0x22,'X','x','Ч','ч'},
																{0x35,'Y','y','Н','н'},
																{0x1A,'Z','z','Я','я'},
																
																{0x29,' ',' ',' ',' '},
																
																{0x45,')','0',')','0'},
																{0x16,'!','1','!','1'},
																{0x1E,'@','2','"','2'},
																{0x26,'#','3','№','3'},
																{0x25,'$','4',';','4'},
																{0x2E,'%','5','%','5'},
																{0x36,'^','6',':','6'},
																{0x3D,'&','7','?','7'},
																{0x3E,'*','8','*','8'},
																{0x46,'(','9','(','9'},
																
																{0x0E,'~','`','Ё','ё'},
																{0x4E,'_','-','_','-'},
																{0x55,'+','=','+','='},
																{0x5D,'|',0x5C,0x2F,0x5C},
																{0x54,'{','[','Х','х'},
																{0x5B,'}',']','Ъ','ъ'},
																{0x4C,':',';','Ж','ж'},
																{0x52,'"',0x27,'Э','э'},
																{0x41,'<',',','Б','б'},
																{0x49,'>','.','Ю','ю'},
																{0x4A,'?','/',',','.'},
																{0x7C,'*','*','*','*'},
																{0x7B,'-','-','-','-'},
																{0x79,'+','+','+','+'},
																
																{0x70,'0','0','0','0'},
																{0x69,'1','1','1','1'},
																{0x72,'2','2','2','2'},
																{0x7A,'3','3','3','3'},
																{0x6B,'4','4','4','4'},
																{0x73,'5','5','5','5'},
																{0x74,'6','6','6','6'},
																{0x6C,'7','7','7','7'},
																{0x75,'8','8','8','8'},
																{0x7D,'9','9','9','9'},
																	
};
extern const char * CANNodeBuffer[];
const char * MsgId[]={"Set Time",
										  "Set Date",
										  "Blink 1",
										  "Blink 2",
										  "Blink 3",
										  "Blink 4",
										  "CAN browser start",
											"CAN browser stop",
											"CAN get data",
											"CAN stop data",
											"CAN set RTC",
											"CAN set date",
											"CAN timer on_off",
											"CAN timer mode",
											"CAN timer val",
											"hello st",
											"CAN update",	
										  0
};

const char *pTaskName[]={"","LED1","LED2","LED3","LED4","Browser CANNET","CAN Update"};
const char *pTaskAction[]={0,"Задача запущена","Ошибка задачи","Задача выполнена","Задача прервана","Неверная команда",
"CRC OK","UART-STM download","CAN download"
};

/********************************************************************************/
void vLCDPrintCurrent(void const * argument){
		osEvent retval;
		uint16_t temp;
		LCDPrintTypeDef data;
	for(;;)
	{	
		retval=osMessageGet(QueuePrintLCDHandle,osWaitForever);
		data.BuffAction=((LCDPrintTypeDef*)retval.value.p)->BuffAction;
		data.BuffName=((LCDPrintTypeDef*)retval.value.p)->BuffName;
				
		osMutexWait(xCANStructPointHandle,osWaitForever);
			if(current_line==438)
				current_line=218;
			HAL_LTDC_SetAddress_NoReload(&hltdc, (LCD_LAYER0_FRAME_BUFFER+4*((current_line-218)*480)), 0);
			HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
			LCD_ClearRec(5,current_line,LCD_XSIZE,22,LAYER_0,BLACK,255);
						
			temp=LCD_PutString(data.BuffAction,5,current_line,0,WHITE,0xFF);
			LCD_PutString(data.BuffName,temp+20,current_line,0,YELLOW,0xFF);
			current_line+=22;
		osMutexRelease(xCANStructPointHandle);
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vPrintKernelLoad(void const * argument){
	char Buffer[]={"   . %"};
	
	uint16_t temp;
	for(;;)
	{
		osSemaphoreWait(xCPULoadSemaphoreHandle,5000);
		
		Buffer[0]=(cpuload/1000)|0x30;
		Buffer[1]=(cpuload%1000)/100|0x30;
		temp=(cpuload%1000)%100;
		Buffer[2]=temp/10|0x30;
		Buffer[4]=temp%10|0x30;
		
		LCD_PutString(Buffer,LCD_XSIZE-310,2,1,MAGENTA,0xFF);
		if(countbyte_firmware)
		{
			Buffer[0]=countbyte_firmware/100000|0x30;
			Buffer[1]=(countbyte_firmware%100000)/10000|0x30;
			Buffer[2]=((countbyte_firmware%100000)%10000)/1000|0x30;
			temp=(countbyte_firmware%100000)%10000;
			Buffer[3]=(temp%1000)/100|0x30;
			Buffer[4]=((temp%1000)%100)/10|0x30;
			Buffer[5]=((temp%1000)%100)%10|0x30;
			LCD_PutString(Buffer,LCD_XSIZE-165,2,1,MAGENTA,0xFF);
			Buffer[3]='.';
			Buffer[5]='%';
		}
		
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vPrintRTC(void const * argument){
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	uint16_t heapfree,heapused,temp;
	char BufferHeap[6];
	char BufferTime[]={"  :  :  "};
	char BufferDate[]={"  .  .  "};
	for(;;)
	{
		osSemaphoreWait (xRTCSemaphoreHandle,osWaitForever);
		
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		BufferTime[0]=(sTime.Hours >> 4)|0x30;
		BufferTime[1]=(sTime.Hours & 0x0F)|0x30;
		BufferTime[3]=(sTime.Minutes >> 4)|0x30;
		BufferTime[4]=(sTime.Minutes & 0x0F)|0x30;
		BufferTime[6]=(sTime.Seconds >> 4)|0x30;
		BufferTime[7]=(sTime.Seconds & 0x0F)|0x30;
		LCD_PutString(BufferTime,LCD_XSIZE-80,2,1,GREEN,0xFF);
		
		BufferDate[0]=(sDate.Date >> 4)|0x30;
		BufferDate[1]=(sDate.Date & 0x0F)|0x30;
		BufferDate[3]=(sDate.Month >> 4)|0x30;
		BufferDate[4]=(sDate.Month & 0x0F)|0x30;
		BufferDate[6]=(sDate.Year >> 4)|0x30;
		BufferDate[7]=(sDate.Year & 0x0F)|0x30;
		LCD_PutString(BufferDate,LCD_XSIZE-470,2,1,GREEN,0xFF);
		
		heapfree=xPortGetFreeHeapSize();
		
		/*BufferHeap[0]=(configTOTAL_HEAP_SIZE/10000)|0x30;
		BufferHeap[1]=(configTOTAL_HEAP_SIZE%10000)/1000|0x30;
		temp=(configTOTAL_HEAP_SIZE%10000)%1000;
		BufferHeap[2]=temp/100|0x30;
		BufferHeap[3]=(temp%100)/10|0x30;
		BufferHeap[4]=(temp%100)%10|0x30;
		BufferHeap[5]=0;
		LCD_PutString(BufferHeap,LCD_XSIZE-55,26,1,CYAN,0xFF);*/
		
		BufferHeap[0]=(heapfree/10000)|0x30;
		BufferHeap[1]=(heapfree%10000)/1000|0x30;
		temp=(heapfree%10000)%1000;
		BufferHeap[2]=temp/100|0x30;
		BufferHeap[3]=(temp%100)/10|0x30;
		BufferHeap[4]=(temp%100)%10|0x30;
		BufferHeap[5]=0;
		LCD_PutString(BufferHeap,LCD_XSIZE-55,26,1,MAGENTA,0xFF);
		
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vLED1Blink(void const * argument){
		uint16_t i;
		LCDPrintTypeDef data;
	for(;;)
	{
		osSemaphoreWait(xLED1SemaphoreHandle,osWaitForever);
				
		data.BuffName=pTaskName[1];
		data.BuffAction=pTaskAction[TASKSTART];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		for(i=0;i<20;i++)
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6,GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6,GPIO_PIN_RESET);
			osDelay(100);
		}
		data.BuffName=pTaskName[1];
		data.BuffAction=pTaskAction[TASKREADY];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		
		//osSemaphoreWait(xLED1SemaphoreHandle,osWaitForever);
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vLED2Blink(void const * argument){
		uint16_t i;
		LCDPrintTypeDef data;
	for(;;)
	{
		osSemaphoreWait(xLED2SemaphoreHandle,osWaitForever);
		
		data.BuffName=pTaskName[2];
		data.BuffAction=pTaskAction[TASKSTART];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		for(i=0;i<20;i++)
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);
			osDelay(200);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);
			osDelay(200);
		}
		
		data.BuffName=pTaskName[2];
		data.BuffAction=pTaskAction[TASKREADY];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		//osSemaphoreWait(xLED2SemaphoreHandle,osWaitForever);
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vLED3Blink(void const * argument){
	uint16_t i;
	LCDPrintTypeDef data;
	for(;;)
	{
		osSemaphoreWait(xLED3SemaphoreHandle,osWaitForever);
		
		data.BuffName=pTaskName[3];
		data.BuffAction=pTaskAction[TASKSTART];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);	
		for(i=0;i<20;i++)
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET);
			osDelay(200);
		}
		
		data.BuffName=pTaskName[3];
		data.BuffAction=pTaskAction[TASKREADY];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		
		//osSemaphoreWait(xLED3SemaphoreHandle,osWaitForever);
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vLED4Blink(void const * argument){
	uint16_t i;
	LCDPrintTypeDef data;
	for(;;)
	{
		osSemaphoreWait(xLED4SemaphoreHandle,osWaitForever);
		
		
		data.BuffName=pTaskName[4];
		data.BuffAction=pTaskAction[TASKSTART];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		for(i=0;i<20;i++)
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET);
			osDelay(200);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET);
			osDelay(100);
		}
		
		data.BuffName=pTaskName[4];
		data.BuffAction=pTaskAction[TASKREADY];
		osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		
		//osSemaphoreWait(xLED4SemaphoreHandle,osWaitForever);
	}
	osThreadTerminate (NULL);
}
/*
*
*/
/*******************************************************************************/
 void vLCDBlinkCursor(void const * argument){
	static uint8_t CursorBlink=0;
	 	 
	 for(;;)
	 {
			osSemaphoreWait(xBlinkCursorSemaphoreHandle,osWaitForever);
			
			if(CursorBlink)
			{
				
				CursorBlink=0;
				LCD_ClearRec(CursorPos,267,CURSOR_WIDTH,CURSOR_HEIGHT,LAYER_1,BLACK,255);
			}
			else
			{	
				
				CursorBlink=1;
				LCD_ClearRec(CursorPos,267,CURSOR_WIDTH,CURSOR_HEIGHT,LAYER_1,MAGENTA,255);
			}
			
			
		}
 	 osThreadTerminate (NULL);
	 
}
 
/*******************************************************************************/
void vApplicationIdleHook(void){
		static uint32_t count=0;
		static uint32_t maxcount=0;
		static uint32_t lasttick=0;
	count++;
	if((osKernelSysTick ()-lasttick)>1000)
	{	
		lasttick=osKernelSysTick ();
		if(count>maxcount)	
			maxcount=count;	
		cpuload=1000-(1000*count)/maxcount;
		count=0;
		osSemaphoreRelease(xCPULoadSemaphoreHandle);
		osSemaphoreRelease(xBlinkCursorSemaphoreHandle);
	}
}


/********************************************************************************/
void vPS2CharDecode(void const * argument){
	uint8_t temp,temp1,i;
	static uint8_t count=0;
	static uint8_t lightskey=0;
	for(;;)
	{
		osSemaphoreWait(xPS2CharSemaphoreHandle,osWaitForever);
	
		temp=PS2_KBD_VAR.buff[0];
		temp1=PS2_KBD_VAR.buff[1];
		if(temp==0xE0)
		{
			switch(temp1){
				case ALT:
					ControlKey.alt_press=1;
				break;	
				case CTRL:
					ControlKey.ctrl_press=1;
				break;	
				case ENTER:
					CommandMsgBuffer[count]=0;
					LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
					count=0;
					CursorPos=5;
					osSemaphoreRelease(xInputMsgSemaphoreHandle);
				break;
				case DIV:
					if(CursorPos<=452)
					{
						CommandMsgBuffer[count]=PS2Buff[47].latLower;
						count++;
						LCD_ClearRec(CursorPos,267,CURSOR_WIDTH,CURSOR_HEIGHT,LAYER_1,BLACK,255);
						CursorPos=LCD_PutChar(PS2Buff[47].latLower,CursorPos,248,1,WHITE);
					}
				break;		
			}
		}
		else
		{
			switch(temp){
				case TAB:
					if(CursorPos>450)
						break;
					LCD_ClearRec(CursorPos,267,CURSOR_WIDTH,CURSOR_HEIGHT,LAYER_1,BLACK,255);
					*(uint32_t*)(CommandMsgBuffer+count)=(uint32_t)0x20202020;
					count+=4;
					CursorPos+=20;
				break;
				case NUMLOCK:
					if(PS2_KBD_Tx(SET_KEYBOARD_INDICATORS)!=SUCCESS)
						break;
					if(ControlKey.numlock_on)
					{
						lightskey &=~KEYBOARD_NUM_LOCK_ON;
						if(PS2_KBD_Tx(lightskey)!=SUCCESS)
							break;
						ControlKey.numlock_on=0;
					}	
					else
					{
						lightskey |=KEYBOARD_NUM_LOCK_ON;
						if(PS2_KBD_Tx(lightskey)!=SUCCESS)
							break;
						ControlKey.numlock_on=1;
					}
				break;		
				case CAPSLOCK:
					if(PS2_KBD_Tx(SET_KEYBOARD_INDICATORS)!=SUCCESS)
						break;
					if(ControlKey.capslock_on)
					{
						lightskey &=~KEYBOARD_CAPS_LOCK_ON;
						if(PS2_KBD_Tx(lightskey)!=SUCCESS)
							break;
						ControlKey.capslock_on=0;
					}
					else
					{
						lightskey |=KEYBOARD_CAPS_LOCK_ON;
						if(PS2_KBD_Tx(lightskey)!=SUCCESS)
							break;
						ControlKey.capslock_on=1;
					}
				break;
				case SHIFT_L:
						if((ControlKey.alt_press)&&(ControlKey.EngRus))
						{	
							LCD_PutString("EN",LCD_XSIZE-370,2,1,MAGENTA,0xFF);
							ControlKey.EngRus=0;
						}
						else if((ControlKey.alt_press)&&(ControlKey.EngRus==0))
						{
							LCD_PutString("RU",LCD_XSIZE-370,2,1,MAGENTA,0xFF);	
							ControlKey.EngRus=1;
						}
						ControlKey.shift_press=1;		
				break;
				case SHIFT_R:
						if((ControlKey.alt_press)&&(ControlKey.EngRus))
						{	
							LCD_PutString("EN",LCD_XSIZE-370,2,1,MAGENTA,0xFF);
							ControlKey.EngRus=0;
						}
						else if((ControlKey.alt_press)&&(ControlKey.EngRus==0))
						{
							LCD_PutString("RU",LCD_XSIZE-370,2,1,MAGENTA,0xFF);	
							ControlKey.EngRus=1;
						}
						ControlKey.shift_press=1;
				break;	
				case CTRL:
					ControlKey.ctrl_press=1;
				break;
				case ALT:
					ControlKey.alt_press=1;
				break;
				case BACKSPACE:
					if(CursorPos<=5)
						break;
					count--;
					i=CommandMsgBuffer[count];
					if(i < 0x80)
						i-=0x1e;
					else
						i-=0x3e;
					temp=GUI_FontArialNarrow22_CharInfo[i].XDist+GUI_FontArialNarrow22_CharInfo[i].BytesPerLine/2;
					CursorPos-=temp;
					LCD_ClearRec(CursorPos,248,temp+CURSOR_WIDTH,22,LAYER_1,BLACK,255);
					CommandMsgBuffer[count]=0;
				break;
				case ENTER:
					CommandMsgBuffer[count]=0;
					LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
					count=0;
					CursorPos=5;
					osSemaphoreRelease(xInputMsgSemaphoreHandle);
				break;
				default:
					i=0;
					while(temp!= PS2Buff[i].scancode)
					{
						i++;
						if(i>countof(PS2Buff))	break;
					}
					
					if((i<=countof(PS2Buff))&&(CursorPos<452))
					{
						if((ControlKey.shift_press)||(ControlKey.capslock_on))
						{
							if(ControlKey.EngRus)
								temp=PS2Buff[i].cirCapital;
							else
								temp=PS2Buff[i].latCapital;
						}
						else 
						{
							if(ControlKey.EngRus)
								temp=PS2Buff[i].cirLower;
							else
								temp=PS2Buff[i].latLower;	
						}
						CommandMsgBuffer[count]=temp;
						count++;
						LCD_ClearRec(CursorPos,267,CURSOR_WIDTH,CURSOR_HEIGHT,LAYER_1,BLACK,255);
						CursorPos=LCD_PutChar(temp,CursorPos,248,1,WHITE);
					}
				break;	
			}				
		
		}
		
	}
	osThreadTerminate (NULL);
}
/********************************************************************************/
void vCommandMsgDecode(void const * argument){
	uint8_t i;
	LCDPrintTypeDef data;
	static uint8_t task_second_param=0;
	static TaskIdTypeDef TaskId;
	CANTX_TypeDef canTX;
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
	for(;;)
	{
		osSemaphoreWait(xInputMsgSemaphoreHandle,osWaitForever);
		i=0;
		while(strcmp(MsgId[i],(const char*)CommandMsgBuffer))
		{
			i++;
			if(MsgId[i]==0)
				break;
		}
		switch(i){
			case 0:	//"Set Time"
				if(TaskId.Id)
					break;
				task_second_param=1;
				CursorPos=LCD_PutString("Введите <<часы>> : <<минуты>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
			break;	
			case 1:	// "Set Date"
				if(TaskId.Id)
					break;
				task_second_param=2;
				CursorPos=LCD_PutString("Введите <<дата>>.<<месяц>>.<<год>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
			break;
			case 2:	 // "мама включи 1"
				osSemaphoreRelease(xLED1SemaphoreHandle);
			break;	
			case 3:	 // "мама включи 2"
				osSemaphoreRelease(xLED2SemaphoreHandle);
			break;	
			case 4:	 // "мама включи 3"
				osSemaphoreRelease(xLED3SemaphoreHandle);
			break;	
			case 5:	 // "мама включи 4"
				osSemaphoreRelease(xLED4SemaphoreHandle);
			break;
			case 6:	//"CAN browser start"
				if(CANNetID)
					break;
				if(xTaskCreate(vTaskCANTX,MsgId[i],128,&TaskId,osPriorityHigh,&CANTX)!=pdTRUE)
					break;
				if(xTaskCreate(vTaskCANFIFO,MsgId[i],128,&TaskId,osPriorityAboveNormal,&CANNetID)!=pdTRUE)
					break;
				if(xTaskCreate(vTimer2000ms,MsgId[i],128,&TaskId,osPriorityNormal,&Timer2000)!=pdTRUE)				
					break;
				__HAL_TIM_ENABLE(&htim6);
				
				data.BuffName=pTaskName[5];
				data.BuffAction=pTaskAction[TASKSTART];
				osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
			break;
			case 7:	//"CAN browser stop"
				if(CANNetID==0)
					break;
				__HAL_TIM_DISABLE(&htim6);
				osThreadTerminate(CANTX);
				osThreadTerminate(CANNetID);
				osThreadTerminate(Timer2000);
				CANTX=0;
				Timer2000=0;
				CANNetID=0;
				vPortFree(pCANNode);
				pCANNode=NULL;
				
				data.BuffName=pTaskName[5];
				data.BuffAction=pTaskAction[TASKREADY];
				osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
				
				LCD_ClearRec(LCD_XSIZE-87,68,85,(22*last_countnode+4),LAYER_1,BLACK,0);
				last_countnode=0;
			break;	
			case 8:	//"CAN get data"
				if(Timer1000)
					break;
				if(xTaskCreate(vTimer1000ms,MsgId[i],128,&TaskId,osPriorityNormal,&Timer1000)!=pdTRUE)				
					break;
				canTX.ID=0x80;
				canTX.typeframe=REMOTEFRAME;
				xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	//широковещательный запрос GET_RTC 
			
				canTX.ID=0x82;
				canTX.typeframe=REMOTEFRAME;
				xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	 //широковещательный запрос GET_TIMER_DATA	
				
				__HAL_TIM_ENABLE(&htim7);
				
			break;
			case 9:	//"CAN stop data"
				if(Timer1000==0)
					break;
				__HAL_TIM_DISABLE(&htim7);
				osThreadTerminate(Timer1000);
				Timer1000=0;
			break;
			case 10:	//"CAN set RTC"
				CursorPos=LCD_PutString("<<Индекс CAN>> : <<часы>> : <<минуты>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
				task_second_param=3;	
			break;		
			case 11:	//"CAN set date"
				CursorPos=LCD_PutString("<<Индекс узла CAN>> : <<дата>>.<<месяц>>.<<год>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
				task_second_param=4;
			break;
			case 12:	//"CAN timer on_off"
				CursorPos=LCD_PutString("<<Индекс узла CAN>> : <<on/off>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
				task_second_param=5;	
			break;
			case 13:	//"CAN timer mode"
				CursorPos=LCD_PutString("<<Индекс узла CAN>> : <<Phas/Brez>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
				task_second_param=6;
			break;
			case 14:	//"CAN timer val"
				CursorPos=LCD_PutString("<<Индекс узла CAN>> : <<Value>>",5,248,1,WHITE,0xFF);
				CursorPos+=10;
				task_second_param=7;
			break;
			case 15:	//"hello st"
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
				UART_Terminal_DMATran("get size");
				task_second_param=8;	
			break;
			case 16:	//"CAN update"
				if(sizebin<1000)
					break;
				countbyte_firmware=0;
				*(uint32_t*)canTX.Data=(uint32_t)sizebin;
				canTX.ID=((indexFirmware+1)<<8)|0x71;   //((canRx.ID+1)<<8)|0x71;
				//indexFirmware=0;
				canTX.DLC=4;
				canTX.typeframe=DATAFRAME;
				xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	// (Core4X9I 0x271) UPDATE_FIRMWARE_REQ 0x271, 0x371, 0x471	
			break;		
			default :
				switch(task_second_param){
					case 0:
						data.BuffName=pTaskName[0];
						data.BuffAction=pTaskAction[COMMANDERR];
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
					break;
					case 1:	//"Set Time" second param
						task_second_param=0;
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						sTime.Hours=(CommandMsgBuffer[0]<<4)|(CommandMsgBuffer[1]&0x0F);
						sTime.Minutes=(CommandMsgBuffer[3]<<4)|(CommandMsgBuffer[4]&0x0F);
						sTime.Seconds=0;
						sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
						sTime.StoreOperation = RTC_STOREOPERATION_RESET;
						if(HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BCD)==HAL_OK)
							data.BuffAction=pTaskAction[TASKREADY];
						else
							data.BuffAction=pTaskAction[TASKERROR];
						data.BuffName=MsgId[0];
					
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
					break;
					case 2:	//"Set Date" second param
						task_second_param=0;
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						sDate.Date=(CommandMsgBuffer[0]<<4)|(CommandMsgBuffer[1]&0x0F);
						sDate.Month=(CommandMsgBuffer[3]<<4)|(CommandMsgBuffer[4]&0x0F);
						sDate.Year=(CommandMsgBuffer[6]<<4)|(CommandMsgBuffer[7]&0x0F);
						sDate.WeekDay=0;
						if(HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BCD)==HAL_OK)
							data.BuffAction=pTaskAction[TASKREADY];
						else
							data.BuffAction=pTaskAction[TASKERROR];
						data.BuffName=MsgId[1];
						
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
					break;
					case 3:		//"CAN set RTC" second param
						task_second_param=0;
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						canTX.ID=(((CommandMsgBuffer[0]&0xF)+1)<<8)|0x81;			// 0x(index+1)81 SET_RTC	
						canTX.DLC=5;
						canTX.Data[0]=(CommandMsgBuffer[2]&0xF)*10;
						canTX.Data[0]+=(CommandMsgBuffer[3]&0xF);
					
						canTX.Data[1]=(CommandMsgBuffer[5]&0xF)*10;
						canTX.Data[1]+=(CommandMsgBuffer[6]&0xF);
					
						HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
						canTX.Data[2]=sDate.Date;
						canTX.Data[3]=sDate.Month;
						canTX.Data[4]=sDate.Year;
						canTX.typeframe=DATAFRAME;					// data frame
						xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	
					break;		
					case 4:
						task_second_param=0;
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
					
					break;
					case 5:	
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						task_second_param=0;
						if(CommandMsgBuffer[3]=='n')
						{
							canTX.ID=(((CommandMsgBuffer[0]&0xF)+1)<<8)|0x82;
							canTX.typeframe=REMOTEFRAME;					// remote frame
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	 // Core4X9I 0x283 remote enable timer
						}
						else if(CommandMsgBuffer[3]=='f')
						{
							canTX.ID=(((CommandMsgBuffer[0]&0xF)+1)<<8)|0x83;
							canTX.typeframe=REMOTEFRAME;					// remote frame
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	 // Core4X9I 0x283 remote disable timer
						}		
						else
						{
							data.BuffName=pTaskName[0];
							data.BuffAction=pTaskAction[COMMANDERR];
							osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
						}
					break;
					case 6:
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						task_second_param=0;
						i=0;
						while(NetIndexBuff[i]!=(CommandMsgBuffer[0]&0xF))
						{
							i++;
							if(i>16) break;
						}
						canTX.ID=(((CommandMsgBuffer[0]&0xF)+1)<<8)|0x83;			// 0x(index+1)83 SET_TIMER_DATA	
						canTX.DLC=4;
						canTX.typeframe=DATAFRAME;		//data frame
						if(CommandMsgBuffer[2]=='P')
						{
							canTX.Data[1]=0;
							canTX.Data[2]=((CANNode_TypeDef*)pCANNode+i)->PhasVal;
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);
						}
						else if(CommandMsgBuffer[2]=='B')
						{
							canTX.Data[1]=1;
							canTX.Data[3]=((CANNode_TypeDef*)pCANNode+i)->BrezVal;
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);
						}
						else
						{
							data.BuffName=pTaskName[0];
							data.BuffAction=pTaskAction[COMMANDERR];
							osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
						}
					break;
					case 7:
						LCD_ClearRec(5,248,(CursorPos+CURSOR_WIDTH),22,LAYER_1,BLACK,255);
						CursorPos=5;
						task_second_param=0;
						i=0;
						while(NetIndexBuff[i]!=(CommandMsgBuffer[0]&0xF))
						{
							i++;
							if(i>16) break;
						}
						canTX.ID=(((CommandMsgBuffer[0]&0xF)+1)<<8)|0x83;			// 0x(index+1)83 SET_TIMER_DATA	
						canTX.DLC=4;
						canTX.typeframe=DATAFRAME;  // data frame
						if(((CANNode_TypeDef*)pCANNode+i)->timerMode==1)
						{
							canTX.Data[1]=1;
							canTX.Data[3]=(CommandMsgBuffer[2]&0xF)*10;
							canTX.Data[3]+=(CommandMsgBuffer[3]&0xF);
						}
						else
						{
							canTX.Data[1]=0;
							canTX.Data[2]=(CommandMsgBuffer[2]&0xF)*10;
							canTX.Data[2]+=(CommandMsgBuffer[3]&0xF);
						}
						xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);
						
					break;
					case 8:
						task_second_param=0;
						if(UART_Download)
							break;
						if(xTaskCreate(vTaskUART_Download,MsgId[i],128,&TaskId,osPriorityAboveNormal,&UART_Download)!=pdTRUE)
							break;
					break;
				}
			break;	
		}
		
	osMutexWait(xDMA2Stream2Handle,osWaitForever);
		HAL_DMA_Abort(&hdma_usart1_rx);
		HAL_DMA_Start(&hdma_usart1_rx,(uint32_t)&USART1->DR,(uint32_t)&CommandMsgBuffer[0],MSGBUFF_SIZE);
	osMutexRelease(xDMA2Stream2Handle);
	
	}
	osThreadTerminate (NULL);
}	
/********************************************************************************/
/*
*
*/
/*********************************************************************************/
void vTimer2000ms(void * pvParameters){
	uint8_t temp,i;
	uint32_t color;
	CANTX_TypeDef canTX;	
		
	for(;;){
			
			osSemaphoreWait(xTIM6SemaphoreHandle,osWaitForever);
			if((TIM6->CR1&TIM_CR1_CEN)!=TIM_CR1_CEN)
			{
				LCD_ClearRec(LCD_XSIZE-85,70,83,22*last_countnode,LAYER_1,BLACK,0);
				last_countnode=0;
			}
			else	
			{
				if((last_countnode!=countnode)||GlobalUpdateFlag)
					{
						NetIndexBuff[countnode]=0;
						taskENTER_CRITICAL();
							vPortFree(pCANNode);
							pCANNode=pvPortMalloc(countnode*sizeof(CANNode_TypeDef));
						taskEXIT_CRITICAL();
						LCD_ClearRec(LCD_XSIZE-87,68,85,(22*last_countnode+4),LAYER_1,BLACK,0);
						i=0;
						while(NetIndexBuff[i])
						{
							if(NetIndexBuff[i]==indexFirmware)
								color=RED;
							else
								color=GREEN;
							LCD_PutString(CANNodeBuffer[NetIndexBuff[i]],LCD_XSIZE-80,70+i*20,1,color,0xFF);
							i++;
						}
						if(countnode)
						{
							LCD_ClearRec(LCD_XSIZE-87,68,85,2,LAYER_1,YELLOW,180);
							LCD_ClearRec(LCD_XSIZE-87,70,2,22*i,LAYER_1,YELLOW,180);
							LCD_ClearRec(LCD_XSIZE-87,(70+22*i),85,2,LAYER_1,YELLOW,180);
						}
						last_countnode=countnode;
						GlobalUpdateFlag=0;
					}
					countnode=0;
					canTX.ID=0x088;
					canTX.typeframe=REMOTEFRAME; // remote frame
					
					xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	//широковещательный запрос get net name
					 
			}
		}
}
/**********************************************************************************/
void vTimer1000ms(void * pvParameters){
	char CANDataArray[]={"             :      .  .    Timer             Val    "};
	uint8_t i;
	CANTX_TypeDef canTX;
	
	for(;;){
			osSemaphoreWait(xTIM7SemaphoreHandle,osWaitForever);
			i=0;
			while(NetIndexBuff[i])
			{
				strcpy(CANDataArray,CANNodeBuffer[NetIndexBuff[i]]);
				CANDataArray[8]=' ';
			
				CANDataArray[11]=((CANNode_TypeDef*)pCANNode+i)->time[0]/10|0x30;
				CANDataArray[12]=((CANNode_TypeDef*)pCANNode+i)->time[0]%10|0x30;
				CANDataArray[14]=((CANNode_TypeDef*)pCANNode+i)->time[1]/10|0x30;
				CANDataArray[15]=((CANNode_TypeDef*)pCANNode+i)->time[1]%10|0x30;
				
				CANDataArray[18]=((CANNode_TypeDef*)pCANNode+i)->date[0]/10|0x30;
				CANDataArray[19]=((CANNode_TypeDef*)pCANNode+i)->date[0]%10|0x30;
				CANDataArray[21]=((CANNode_TypeDef*)pCANNode+i)->date[1]/10|0x30;
				CANDataArray[22]=((CANNode_TypeDef*)pCANNode+i)->date[1]%10|0x30;
				CANDataArray[24]=((CANNode_TypeDef*)pCANNode+i)->date[2]/10|0x30;
				CANDataArray[25]=((CANNode_TypeDef*)pCANNode+i)->date[2]%10|0x30;
				if(((CANNode_TypeDef*)pCANNode+i)->timer0nOff)
				{
					CANDataArray[34]='O';
					CANDataArray[35]='n';
					CANDataArray[36]=' ';
				}
				else
				{
					CANDataArray[34]='O';
					CANDataArray[35]='f';
					CANDataArray[36]='f';
				}
				if(((CANNode_TypeDef*)pCANNode+i)->timerMode)
				{
					CANDataArray[39]='B';
					CANDataArray[40]='r';
					CANDataArray[41]='e';
					CANDataArray[42]='z';
					CANDataArray[50]=((CANNode_TypeDef*)pCANNode+i)->BrezVal /100|0x30;
					CANDataArray[51]=((CANNode_TypeDef*)pCANNode+i)->BrezVal/10|0x30;
					CANDataArray[52]=((CANNode_TypeDef*)pCANNode+i)->BrezVal%10|0x30;
				}
				else
				{
					CANDataArray[39]='P';
					CANDataArray[40]='h';
					CANDataArray[41]='a';
					CANDataArray[42]='s';
					CANDataArray[50]=((CANNode_TypeDef*)pCANNode+i)->PhasVal /100|0x30;
					CANDataArray[51]=((CANNode_TypeDef*)pCANNode+i)->PhasVal/10|0x30;
					CANDataArray[52]=((CANNode_TypeDef*)pCANNode+i)->PhasVal%10|0x30;
				}
				osMutexWait(xCANStructPointHandle,osWaitForever);
					if(current_line==438)
						current_line=218;
					HAL_LTDC_SetAddress_NoReload(&hltdc, (LCD_LAYER0_FRAME_BUFFER+4*((current_line-218)*480)), 0);
					HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
					LCD_ClearRec(0,current_line,LCD_XSIZE,22,LAYER_0,BLACK,255);
					LCD_PutString(CANDataArray,5,current_line,0,WHITE,0xFF);
					current_line+=22;
				osMutexRelease(xCANStructPointHandle);
				i++;	
			}
			
			canTX.ID=0x80;
			canTX.typeframe=REMOTEFRAME;
			xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);		//широковещательный запрос GET_RTC 
			canTX.ID=0x82;
			canTX.typeframe=REMOTEFRAME;
			xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	 //широковещательный запрос GET_TIMER_DATA		 
			
	}
	vTaskDelete(NULL);
}
/***************************************************************************************/

/*******************************************************************************************/