
#ifndef __MYTASK_H
#define __MYTASK_H

#include "cmsis_os.h"

#define MSGBUFF_SIZE  50

#define TASKSTART 		1
#define TASKERROR		 	2 
#define TASKREADY 		3
#define TASKABORT 		4
#define COMMANDERR 		5
#define SETTIME		 		6
#define SETDATE		 		7
#define STARTCAN			8
#define STOPCAN		 		9

#define TYPEMSG				1
#define TYPEDATA			2

extern osSemaphoreId xCPULoadSemaphoreHandle;

extern osSemaphoreId xLED1SemaphoreHandle;
extern osSemaphoreId xLED2SemaphoreHandle;
extern osSemaphoreId xLED3SemaphoreHandle;
extern osSemaphoreId xLED4SemaphoreHandle;

extern osSemaphoreId xInputMsgSemaphoreHandle;
extern osSemaphoreId xBlinkCursorSemaphoreHandle;

extern osSemaphoreId xRTCSemaphoreHandle;
extern osSemaphoreId xDMA2DSemaphoreHandle;
extern osSemaphoreId xDMA2Stream2SemaphoreHandle;
extern osSemaphoreId xDMA2Stream7SemaphoreHandle;
extern osSemaphoreId xCAN_TXSemaphoreHandle;

extern osSemaphoreId xPS2CharSemaphoreHandle;

extern osSemaphoreId xTIM6SemaphoreHandle;
extern osSemaphoreId xTIM7SemaphoreHandle;


extern osMutexId xCANStructPointHandle;
extern  osMutexId xDMA2Stream2Handle;


extern osMessageQId QueuePrintLCDHandle;
extern osMessageQId QueueCANRecieveHandle;
extern osMessageQId QueueCANTransmiteHandle;

extern xQueueHandle xQueueCANTX,xQueueCANRX;

extern char CommandMsgBuffer[];

typedef struct{
	const	char 	*BuffAction;
	const	char  *BuffName;
}LCDPrintTypeDef;


typedef struct{
	uint8_t taskParam;
	osThreadId Id;
}TaskIdTypeDef;

#endif
