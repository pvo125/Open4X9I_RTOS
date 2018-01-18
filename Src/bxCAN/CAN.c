#include <stm32f4xx.h>
#include "CAN.h"
#include "cmsis_os.h"
#include "mytask.h"
#include "string.h"
#include "LCDdriver.h"

extern xTaskHandle UART_Download;


extern const char *pTaskAction[];
extern const char *pTaskName[];


extern volatile uint8_t message_mode;

volatile uint8_t GlobalUpdateFlag=0;
volatile uint8_t indexFirmware=0;
volatile uint32_t sizebin=0;
volatile uint32_t countbyte_firmware=0;


extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;


CANNode_TypeDef *pCANNode=NULL;
uint8_t NetIndexBuff[16];
volatile uint8_t countnode;

const char * CANNodeBuffer[]={"Open4x9I","Core4x9I","F103_KIT","32VLDisc"};
/***************************************************************************************************************/
const uint32_t crc32_table[256]={	
		0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

uint32_t crc32_check(const uint8_t *buff,uint32_t count){
	uint32_t crc=0xffffffff;
	while(count--)
		crc=(crc>>8)^crc32_table[(crc^*buff++) & 0xFF];
	return crc^0xffffffff;
}

/****************************************************************************************************************
*														bxCAN_Init
****************************************************************************************************************/
void bxCAN_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	/*Включаем тактирование CAN в модуле RCC*/	
	RCC->APB1ENR|=RCC_APB1ENR_CAN2EN|RCC_APB1ENR_CAN1EN;
	/*Настройка выводов CAN  CAN2_TX=PA12   CAN2_RX=PA11  */
	
	GPIO_InitStruct.Alternate=GPIO_AF9_CAN2;
	GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin=GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	CAN2->RF1R|=CAN_RF0R_RFOM0;
	CAN2->RF1R|=CAN_RF1R_RFOM1;
	
	/*Настройка NVIC для bxCAN interrupt*/
	HAL_NVIC_SetPriority( CAN2_RX0_IRQn, 6, 0);
	HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 6, 0);
	HAL_NVIC_SetPriority(CAN2_TX_IRQn, 6, 0);

//			Init mode				//

	//CAN2->MCR|=CAN_MCR_RESET;
	
	/*Exit SLEEP mode*/
	CAN1->MCR&=~CAN_MCR_SLEEP;
	CAN2->MCR&=~CAN_MCR_SLEEP;
	/*Enter Init mode bxCAN*/
	CAN1->MCR|=CAN_MCR_INRQ;
	CAN2->MCR|=CAN_MCR_INRQ;  /*Initialization Request */
	while((CAN2->MSR&CAN_MSR_INAK)!=CAN_MSR_INAK)		{}   /*while Initialization Acknowledge*/

	CAN2->MCR|=CAN_MCR_DBF;			// CAN работает во время отладки
	CAN2->MCR|=CAN_MCR_ABOM;
	CAN2->MCR&=~CAN_MCR_TTCM;
	CAN2->MCR&=~CAN_MCR_AWUM;
	CAN2->MCR&=~CAN_MCR_NART;		// сброс запрета автоматич. повторной передачи ( CAN  будет повторно передавать )	
	CAN2->MCR&=~CAN_MCR_RFLM;
	CAN2->MCR&=~CAN_MCR_TXFP;		//	сообщения отправляются в порядке приоритете идентификатора
	/*Тестовый режиим работы выключен CAN  SILM=0  LBKM=0 */
	
	CAN2->BTR&=~CAN_BTR_LBKM;	
	CAN2->BTR&=~CAN_BTR_SILM;	

	CAN2->BTR|=CAN_BTR_BRP&29;														/* tq=(29+1)*tPCLK1=2/3 uS   */
	CAN2->BTR|=CAN_BTR_SJW_0;															/*SJW[1:0]=1  (SJW[1:0]+1)*tCAN=tRJW PROP_SEG =+- 2* tq	*/		
	//CAN2->BTR&=~CAN_BTR_TS1_0;
	CAN2->BTR|=CAN_BTR_TS1_2;															/* TS1[3:0]=0X07 */ //tBS1=tq*(7+1)=8tq
	//CAN2->BTR&=~CAN_BTR_TS2_1;	
	//CAN2->BTR|=CAN_BTR_TS2_0;				  									/* TS2[2:0]=0X02 */ //tBS2=tq*(2+1)=3tq
																												// | 1uS |  	8uS 					 |  3uS			| 		T=12*tq=12*2/3=8us f=125kHz
																												// |-------------------------|----------|		
																												//				Sample point = 75%
	/*Init filters*/
	CAN1->FMR|=	CAN_FMR_FINIT;																		// Filter Init Mode
		
	CAN1->FM1R&=~(CAN_FM1R_FBM14|CAN_FM1R_FBM15);												// Filters bank 14 15 mode ID mask
	CAN1->FM1R|=CAN_FM1R_FBM16;  																				// Filters bank 16  mode ID List
	CAN1->FS1R&=~(CAN_FS1R_FSC14|CAN_FS1R_FSC15|CAN_FS1R_FSC16);				// Filters bank 14 15 16  scale 16 bits
	CAN1->FFA1R&=~(CAN_FFA1R_FFA14|CAN_FFA1R_FFA15|CAN_FFA1R_FFA16);		// Filters bank 14 15 16  FIFO0		
		
	CAN1->FM1R|=CAN_FM1R_FBM17;																					// Filters bank 17      mode ID List		
	CAN1->FM1R&=~	(CAN_FM1R_FBM18|CAN_FM1R_FBM19);											// Filters bank 18  19   mode ID mask
	CAN1->FS1R&=~(CAN_FS1R_FSC17|CAN_FS1R_FSC18|CAN_FS1R_FSC19);				// Filters bank 17 18 19  scale 16 bits	
	CAN1->FFA1R|=CAN_FFA1R_FFA17|CAN_FFA1R_FFA18|CAN_FFA1R_FFA19;				// Filters bank 17 18 19  FIFO1		

	/*ID filters */
  //FOFO0
	CAN1->sFilterRegister[14].FR1=0x1FFF3000;	//Filters bank 14 fmi=00 ID=0xX80 IDE=0 RTR=0	фильтр для сообщ(0x180,0x280,0x380.)на запрос GET_RTC 0x080
																						//							 `  
	CAN1->sFilterRegister[14].FR2=0x30303020;	//Filters bank 14 fmi=01 ID=0x181 IDE=0 RTR=0	 
																						//		  				 `
	CAN1->sFilterRegister[15].FR1=0x1FFF3040;	//Filters bank 15 fmi=02 ID=0xX82 IDE=0 RTR=0	фильтр для сообщ(0x182,0x282,0x382.)на запрос GET_TIMER_DATA 0x082
																						//							 `
	CAN1->sFilterRegister[15].FR2=0x30703060;	//Filters bank 15 fmi=03 ID=0x183 IDE=0 RTR=0	 
																						//							 `fmi=04 ID=0x183 IDE=0 RTR=1
	CAN1->sFilterRegister[16].FR1=0x30903080;	//Filters bank 16 fmi=05 ID=0x184 IDE=0 RTR=0
																						//							 `fmi=06 ID=0x184 IDE=0 RTR=1
	CAN1->sFilterRegister[16].FR2=0x30B030A0;	//Filters bank 16 fmi 07 ID=0x185 IDE=0 RTR=0	
																						//							 `fmi=08 ID=0x185 IDE=0 RTR=1
	
	//FIFO1  
	CAN1->sFilterRegister[17].FR1=0x30D030C0;	//Filters bank 17 fmi=00 ID=0x186 IDE=0 RTR=0	
																						//								fmi=01 ID=0x186 IDE=0 RTR=1
	CAN1->sFilterRegister[17].FR2=0x30F030E0;	//Filters bank 17 fmi=02 ID=0x187 IDE=0 RTR=0	 
																						//								fmi=03 ID=0x187 IDE=0 RTR=1
	CAN1->sFilterRegister[18].FR1=0x1FFF3100;	//Filters bank 18 fmi=04 ID=0xX88 IDE=0 RTR=0	фильтр для сообщ(0x188,0x288,0x388..)на запрос GET_NETNAME 0x088
																						//								
	CAN1->sFilterRegister[18].FR2=0x1FFF4E40;	//Filters bank 18 fmi=05 ID=0xX72 IDE=0 RTR=0	фильтр для сообщ(0x172,0x272,0x372...) на запрос 0xX71 UPDATE_FIRMWARE_REQ 
																						// 								
	CAN1->sFilterRegister[19].FR1=0x1FFF4E80;	//Filters bank 19 fmi=06 ID=0xX74 IDE=0 RTR=0	фильтр для 0xX74 (0x174) 0x274 0x374) запрос от bootloader GET_FIRMWARE	
																						//								
	CAN1->sFilterRegister[19].FR2=0x31703160;	//Filters bank 19 fmi=08 ID=0x18B IDE=0 RTR=0
																						// 								fmi=09 ID=0x18B IDE=0 RTR=1	
	/* Filters activation  */	
	CAN1->FA1R|=CAN_FFA1R_FFA14|CAN_FFA1R_FFA15|CAN_FFA1R_FFA16|
							CAN_FFA1R_FFA17|CAN_FFA1R_FFA18|CAN_FFA1R_FFA19;		
							
	/*Exit filters init mode*/
	CAN1->FMR&=	~CAN_FMR_FINIT;
	
	/*Разрешение прерываний FIFO0 FIFO1 TMEIE */
	CAN2->IER|=CAN_IER_FMPIE0|CAN_IER_FMPIE1|CAN_IER_TMEIE;
	
//	 Exit Init mode bxCAN	

	CAN2->MCR&=~CAN_MCR_INRQ;  														/*Initialization Request */	
	while((CAN2->MSR&CAN_MSR_INAK)==CAN_MSR_INAK)		{}   /*while Initialization Acknowledge*/		

	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);		
	HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);	
}
/*****************************************************************************************************************
*													Task CAN_Transmit
******************************************************************************************************************/
void vTaskCANTX(void * pvPareters){
	uint8_t mailbox_index;
	CANTX_TypeDef canTX;
	portBASE_TYPE xStatus;
	for(;;)	
	{
		xStatus=xQueueReceive(xQueueCANTX,&canTX,osWaitForever);
		
		if((CAN2->TSR&CAN_TSR_TME0)==CAN_TSR_TME0)
			mailbox_index=0;
		else if((CAN2->TSR&CAN_TSR_TME1)==CAN_TSR_TME1)
			mailbox_index=1;
		else if((CAN2->TSR&CAN_TSR_TME2)==CAN_TSR_TME2)
		{
			mailbox_index=2;
			osSemaphoreWait(xCAN_TXSemaphoreHandle,osWaitForever);
		}
		else
		{
			osSemaphoreWait(xCAN_TXSemaphoreHandle,osWaitForever);
			mailbox_index=2;
		}
		if(canTX.typeframe==REMOTEFRAME)						// remote frame;
		{
			CAN2->sTxMailBox[mailbox_index].TDTR&=(uint32_t)0xfffffff0;
			CAN2->sTxMailBox[mailbox_index].TIR=(canTX.ID <<21)|0x2|CAN_TI0R_TXRQ;
		}
		else																				// data frame	
		{
			CAN2->sTxMailBox[mailbox_index].TDTR&=(uint32_t)0xfffffff0;
			CAN2->sTxMailBox[mailbox_index].TDTR|=canTX.DLC;
			CAN2->sTxMailBox[mailbox_index].TDLR=*(uint32_t*)canTX.Data;
			CAN2->sTxMailBox[mailbox_index].TDHR=*(uint32_t*)(canTX.Data+4);
			
			CAN2->sTxMailBox[mailbox_index].TIR=canTX.ID<<21|CAN_TI0R_TXRQ;
		}
	}
	vTaskDelete(NULL);
}

/*****************************************************************************************************************
*													Task CAN_FIFO
*****************************************************************************************************************/
void vTaskCANFIFO(void *pvParameters){
	char buff[]="    %";
	portBASE_TYPE xStatus;
	uint8_t i;
	uint32_t temp;
	LCDPrintTypeDef data;
	CANRX_TypeDef canRX;
	CANTX_TypeDef canTX;
	for(;;)	
	{
		xStatus=xQueueReceive(xQueueCANRX,&canRX,osWaitForever);
		canRX.ID>>=8;
		if(canRX.FIFO)
		{	switch(canRX.FMI){
				case 4:	//(id=x88 data get netname ответы с net name)
					NetIndexBuff[countnode]=(uint8_t)canRX.ID;
					countnode++;
				break;
				
				case 5:	//(id=0xX72 ) ответы на запрос UPDATE_FIRMWARE_REQ
					if(canRX.Data[1]=='g')		// получили в сообщении 'g' GET_DATA!
					{								
						if((sizebin-countbyte_firmware)>=8) 
						{
							*(uint64_t*)canTX.Data=*(uint64_t*)(SDRAM_UPDATE+countbyte_firmware);
							countbyte_firmware+=8;
							if((countbyte_firmware%2400)==0)
							{
								temp=(countbyte_firmware*100)/sizebin;
								buff[0]=temp/100|0x30;
								buff[1]=(temp%100)/10|0x30;
								buff[2]=(temp%100)%10|0x30;
								data.BuffAction=pTaskAction[8];
								data.BuffName=buff;
								osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
							}
							
							canTX.ID=((canRX.ID+1)<<8)|0x73;
							canTX.DLC=8;
							canTX.typeframe=DATAFRAME;	
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);
						}
						else if((sizebin-countbyte_firmware))
						{
							canTX.DLC=(uint8_t)(sizebin-countbyte_firmware);	
							for(i=0;i<canTX.DLC;i++)
								{
									canTX.Data[i]=*(uint8_t*)(SDRAM_UPDATE+countbyte_firmware);
									countbyte_firmware++;
								}
							temp=(countbyte_firmware*100)/sizebin;
							buff[0]=temp/100|0x30;
							buff[1]=(temp%100)/10|0x30;
							buff[2]=(temp%100)%10|0x30;	
							data.BuffAction=pTaskAction[8];
							data.BuffName=buff;
							osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
								
							canTX.ID=((canRX.ID+1)<<8)|0x73;
							canTX.typeframe=DATAFRAME;		
							xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);
								
						}
					}
					else if(canRX.Data[1]=='c')		// получили в сообщении 'c' CRC OK!
					{
						GlobalUpdateFlag=1;
						indexFirmware=0;
						countbyte_firmware=0;
						data.BuffAction=pTaskAction[6];
						data.BuffName=pTaskName[6];
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
						LCD_ClearRec(315,2,58,22,LAYER_1,BLACK,255);
					}
					else if(canRX.Data[1]=='e')		// получили в сообщении 'e' CRC ERROR!
					{
						countbyte_firmware=0;
						data.BuffAction=pTaskAction[TASKERROR];
						data.BuffName=pTaskName[6];
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
						LCD_ClearRec(315,2,58,22,LAYER_1,BLACK,255);
					}
					else if(canRX.Data[1]=='s')		// получили в сообщении 's' SIZE ERROR!
					{
						countbyte_firmware=0;
						data.BuffAction=pTaskAction[TASKERROR];
						data.BuffName=pTaskName[6];
						osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
						LCD_ClearRec(315,2,58,22,LAYER_1,BLACK,255);
					}
				break;
				case 6://(id=x74 get_firmware)
					/*if(pCANNode==NULL)
						break;
					i=0;
					while(NetIndexBuff[i]!=(uint8_t)canRX.ID)
					{
						i++;
						if(i>16) break;
					}
					((CANNode_TypeDef*)pCANNode+i)->UpdateReq=1;*/
					if((canRX.ID)==indexFirmware)
					{
						countbyte_firmware=0;
						*(uint32_t*)canTX.Data=(uint32_t)sizebin;
						canTX.ID=((canRX.ID+1)<<8)|0x71;
						canTX.DLC=4;
						canTX.typeframe=DATAFRAME;
						xQueueSendToBack(xQueueCANTX,&canTX,osWaitForever);	// (Core4X9I 0x271) UPDATE_FIRMWARE_REQ 0x271, 0x371, 0x471	
					}
				break;
			}
		}
		else			//FIFO_0
		{ switch(canRX.FMI){
				case 0:	//(id=x80 data get rtc-time_date)
					if(pCANNode==NULL)
						break;
					i=0;
					while(NetIndexBuff[i]!=(uint8_t)canRX.ID)
					{
						i++;
						if(i>16) break;
					}
					((CANNode_TypeDef*)pCANNode+i)->netIndex=(uint8_t)canRX.ID;
					((CANNode_TypeDef*)pCANNode+i)->time[0]=canRX.Data[1];
					((CANNode_TypeDef*)pCANNode+i)->time[1]=canRX.Data[2];
					
					((CANNode_TypeDef*)pCANNode+i)->date[0]=canRX.Data[3];
					((CANNode_TypeDef*)pCANNode+i)->date[1]=canRX.Data[4];
					((CANNode_TypeDef*)pCANNode+i)->date[2]=canRX.Data[5];
				break;
				case 2://(id=x82 data get_timer)	
					if(pCANNode==NULL)
						break;
					i=0;
					while(NetIndexBuff[i]!=(uint8_t)canRX.ID)
					{
						i++;
						if(i>16) break;
					}
					((CANNode_TypeDef*)pCANNode+i)->netIndex=(uint8_t)canRX.ID;
					((CANNode_TypeDef*)pCANNode+i)->timerMode=canRX.Data[1];
					((CANNode_TypeDef*)pCANNode+i)->PhasVal=canRX.Data[2];
					((CANNode_TypeDef*)pCANNode+i)->BrezVal=canRX.Data[3];
					((CANNode_TypeDef*)pCANNode+i)->timer0nOff=canRX.Data[4];
					((CANNode_TypeDef*)pCANNode+i)->can_err[0]=canRX.Data[7];	//CAN_REC
					((CANNode_TypeDef*)pCANNode+i)->can_err[1]=canRX.Data[6];	//CAN_TEC
					((CANNode_TypeDef*)pCANNode+i)->can_err[2]=canRX.Data[5];	//CAN_ERF
				break;
			}
		}
	}
	vTaskDelete(NULL);
}

/********************************************************************************************************************/
void vTaskUART_Download(void *pvParameters){
	uint8_t i=0,temp1;
	int32_t temp;
	int32_t dec=1;
	uint32_t crc;
	LCDPrintTypeDef data;
	CANTX_TypeDef canTX; 
	char buff[]="    %";	
	
		message_mode=1;
		while(CommandMsgBuffer[i]) i++;
		sizebin=0;
		while(i)
		{	i--;
			sizebin+=(CommandMsgBuffer[i]&0x0f)*dec;
			dec*=10;
		}
		temp=sizebin;
		osMutexWait(xDMA2Stream2Handle,osWaitForever);
			HAL_DMA_Abort(&hdma_usart1_rx);
			HAL_DMA_Start(&hdma_usart1_rx,(uint32_t)&USART1->DR,(uint32_t)SDRAM_UPDATE,UARTDATA_PACKETSIZE);
			//DMA2_Stream2->CR &=~DMA_SxCR_EN;
			//DMA2->LIFCR |=DMA_LIFCR_CTCIF2|DMA_LIFCR_CHTIF2;
			//DMA2_Stream2->NDTR=UARTDATA_PACKETSIZE;
			//DMA2_Stream2->M0AR=SDRAM_UPDATE;
		data.BuffAction=pTaskAction[7];
		while(temp>0)
		{
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
			UART_Terminal_DMATran("get data");			
			DMA2_Stream2->CR |=DMA_SxCR_EN;
			osSemaphoreWait(xDMA2Stream2SemaphoreHandle,osWaitForever);
			DMA2_Stream2->CR &=~DMA_SxCR_EN;
			DMA2->LIFCR |=DMA_LIFCR_CTCIF2|DMA_LIFCR_CHTIF2;
			DMA2_Stream2->M0AR+=(UARTDATA_PACKETSIZE-DMA2_Stream2->NDTR-1);
			temp-=(UARTDATA_PACKETSIZE-DMA2_Stream2->NDTR-1);
				
			temp1=((sizebin-temp)*100)/sizebin;
			buff[0]=temp1/100|0x30;
			buff[1]=(temp1%100)/10|0x30;
			buff[2]=(temp1%100)%10|0x30;
			data.BuffName=buff;
			osMessagePut(QueuePrintLCDHandle,(uint32_t)&data,osWaitForever);
		}
		HAL_DMA_Abort(&hdma_usart1_rx);
		HAL_DMA_Start(&hdma_usart1_rx,(uint32_t)&USART1->DR,(uint32_t)&CommandMsgBuffer[0],MSGBUFF_SIZE);
		osMutexRelease(xDMA2Stream2Handle);
		message_mode=0;
		crc=crc32_check((const uint8_t *)SDRAM_UPDATE,(sizebin-4));
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		if(crc==*(uint32_t*)(SDRAM_UPDATE+sizebin-4))
		{
			UART_Terminal_DMATran("crc ok!");
						
			taskENTER_CRITICAL();
				indexFirmware=0;
				while(strcmp(CANNodeBuffer[indexFirmware],(const char*)(0xD0400020)))
				{
					indexFirmware++;
				}
				GlobalUpdateFlag=1;	
			taskEXIT_CRITICAL();
		}
		else
			UART_Terminal_DMATran("crc error!");
		
		
		
		UART_Download=0;
		vTaskDelete(NULL);
}

/**************************************************************************************************************/
void UART_Terminal_DMATran(char *p){
	uint8_t count=0;	
	
	DMA2_Stream7->PAR=(uint32_t)&USART1->DR;
	DMA2_Stream7->M0AR=(uint32_t)p;
	while((*p++)!=0x0)
		count++;
	DMA2_Stream7->NDTR=count;
	DMA2->HIFCR |=DMA_HIFCR_CTCIF7|DMA_HIFCR_CHTIF7;
	DMA2_Stream7->CR |=DMA_SxCR_TCIE|DMA_SxCR_EN;	
}

/*********************************************************************************************************************/
