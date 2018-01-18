#ifndef _CAN_H_
#define _CAN_H_

#include <stm32f4xx.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


#define CODEFLASH
#define SDRAM_UPDATE					0xD0400000
#define UARTDATA_PACKETSIZE 	20000	

#define DATAFRAME 0
#define REMOTEFRAME 1

typedef enum
{
	CAN_TXOK=0,
	CAN_TXERROR,
	CAN_TXBUSY
} CAN_TXSTATUS;

typedef enum
{
	CAN_RXOK=0,
	CAN_RXERROR,
} CAN_RXSTATUS;


typedef struct
{
	uint8_t typeframe;
	uint16_t ID;
	uint8_t DLC;
	uint8_t Data[8];
} CANTX_TypeDef; 



typedef struct
{
	uint8_t FIFO;
	uint16_t ID;
	uint8_t FMI;
	uint8_t DLC;
	uint8_t Data[8];
} CANRX_TypeDef; 

extern CANTX_TypeDef CAN_Data_TX;

typedef struct
{
	uint8_t netIndex;
	uint8_t UpdateReq;
	uint8_t can_err[3];
	uint8_t timer0nOff;
	uint8_t timerMode;
	uint8_t BrezVal;
	uint8_t PhasVal;
	uint8_t time[2];
	uint8_t date[3];
}CANNode_TypeDef;

typedef struct
{
	uint8_t count;
	//WM_HWIN   hWin;
}CANNodeDialog_Typedef;	

extern volatile uint8_t netname_index;

void bxCAN_Init(void);
void CAN_Transmit(CANTX_TypeDef * p);
//CAN_TXSTATUS CAN_Transmit_DataFrame(CANTX_TypeDef *Data);
//CAN_TXSTATUS CAN_Transmit_RemoteFrame(uint16_t ID);
void CAN_Receive_IRQHandler(uint8_t FIFONumber);

extern void vTaskCANTX(void *pvParameter);
extern void vTaskCANFIFO(void *pvParameter);
extern void vTaskUART_Download(void *pvParameters); 
extern void vTaskUpdateOverCAN(void *pvParameters);

extern void UART_Terminal_DMATran(char *p);

void CAN_RXProcess0(void);
void CAN_RXProcess1(void);










#endif
