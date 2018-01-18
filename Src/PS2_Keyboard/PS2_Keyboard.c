#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "PS2_Keyboard.h"
#include "cmsis_os.h"
#include "mytask.h"
//	PA7 clk    timer
//	PC1 data	 gpio

volatile uint8_t scancode=0;
volatile uint8_t parity;
extern volatile ControlKeyTypeDef ControlKey;

volatile PS2_KBD_VAR_t		PS2_KBD_VAR;

void PS2_KBD_LowLevelInit(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	KBD_DATA_PORT->ODR|=DATA_ODR_ODRx;													
	KBD_DATA_PORT->MODER|=DATA_MODER_MODERx; 						 			// General purpose output mode
	KBD_DATA_PORT->OTYPER|=DATA_OTYPER_OTx;										// Output open-drain
	KBD_DATA_PORT->OSPEEDR|=DATA_OSPEEDER_OSPEEDRx;						// Medium speed
	KBD_DATA_PORT->PUPDR|=DATA_PUPDR_PUPDRx;									// Pull-up
	KBD_DATA_PORT->BSRR=DATA_BSRR_BS;													// линию  data  в :1
	
	KBD_CLOCK_PORT->ODR|=CLOCK_ODR_ODRx;													
	KBD_CLOCK_PORT->MODER&=~CLOCK_MODER_MODERx;
	KBD_CLOCK_PORT->MODER|=CLOCK_MODER_MODERx_1; 				//Alternate function mode	MOUSE_CLOCK_PORT->MODER|=CLOCK_MODER_MODERx; 						  // General purpose output mode
	KBD_CLOCK_PORT->OTYPER|=CLOCK_OTYPER_OTx;								// Output open-drain
	KBD_CLOCK_PORT->OSPEEDR|=CLOCK_OSPEEDER_OSPEEDRx;				// Medium speed
	KBD_CLOCK_PORT->PUPDR|=CLOCK_PUPDR_PUPDRx;							// Pull-up
	KBD_CLOCK_PORT->BSRR=CLOCK_BSRR_BS;																			// clock  в :1
	
	
	KBD_CLOCK_PORT->AFR|=GPIO_AFIO;							//  Настроим мультиплексор на AF2 TIM4. Порт пока в General purpose output mode
	
	
	
	__TIMx_CLK_ENABLE();
	TIMx->CR1|=TIM_CR1_CKD_1 ;								//tDTS = 4 × tCK_INT		
	TIMx->SMCR|=TIM_SMCR_TS_1|TIM_SMCR_TS_2;   //110: Filtered Timer Input 2 (TI2FP2)
	TIMx->SMCR|=TIM_SMCR_SMS;									//111: External Clock Mode 1 	
	TIMx->DIER|=TIM_DIER_TIE|TIM_DIER_UIE;		//<Update interrupt enable  Trigger interrupt enable. Прерывание по спаду и по переполнению TIM	
	TIMx->CCER|=TIM_CCER_CC2P;								// inverted/falling edge    По спаду линии clock
	
	TIMx->CCMR1|=TIM_CCMR1_IC2F_0|
								TIM_CCMR1_IC2F_2|
								TIM_CCMR1_IC2F_3;  				//  1101: fSAMPLING=fDTS/32, N=5   90MHz/4=22,5 MHz fDTS  Цифровой фильтр для PS2 clock.
	
	TIMx->CCMR1|=TIM_CCMR1_CC2S_0;						//	CC2 channel is configured as input, IC2 is mapped on TI2
	
	TIMx->PSC=0;
	TIMx->ARR=10;
	TIMx->EGR|=TIM_EGR_UG;										// Сделаем UEV чтобы ARR обновился
	TIMx->SR&=~TIM_SR_UIF; 										// Сбросим флаг прерывания по UEV
	HAL_NVIC_SetPriority(TIMx_IRQn, 7, 0);		// 
	NVIC_ClearPendingIRQ(TIMx_IRQn);					// 
	NVIC_EnableIRQ(TIMx_IRQn);								// Разрешим прерывание
	TIMx->CR1|=TIM_CR1_CEN;										// Запуск таймера
		
	
  
  PS2_KBD_VAR.receive_code=0;							//	
  PS2_KBD_VAR.send_code=0;								// Обнуляем переменную для отправки и принятия данных. На всякий случай.
	
	
}
/*
*/
void PS2_KBD_LowLevelDeInit(void){
/* Сброс всех битов TIM в начальное состояние	*/
	
	TIMx->CR1&=~TIM_CR1_CEN;										// Выключаем таймер
	TIMx->CR1&=~TIM_CR1_CKD_1 ;									//tDTS = 4 × tCK_INT		
	TIMx->SMCR&=~TIM_SMCR_TS_1|TIM_SMCR_TS_2;   //110: Filtered Timer Input 2 (TI2FP2)
	TIMx->SMCR&=~TIM_SMCR_SMS;									//111: External Clock Mode 1 	
	TIMx->DIER&=~TIM_DIER_TIE|TIM_DIER_UIE;			//<Update interrupt enable  Trigger interrupt enable	
	TIMx->CCER&=~TIM_CCER_CC1P;									// inverted/falling edge
	TIMx->CCMR1&=~TIM_CCMR1_IC2F_0|
								TIM_CCMR1_IC2F_2|
								TIM_CCMR1_IC2F_3;  						//  1101: fSAMPLING=fDTS/32, N=5   90MHz/4=22,5 MHz fDTS  
	TIMx->CCMR1&=~TIM_CCMR1_CC2S_0;							//	CC2 channel is configured as input, IC2 is mapped on TI2
	
	TIMx->PSC=0;
	TIMx->ARR=0;
	HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
	NVIC_ClearPendingIRQ(TIMx_IRQn);
	NVIC_DisableIRQ(TIMx_IRQn);
	__TIMx_CLK_DISABLE();																						// Выключаем тактирование таймера
	
	
																					
	KBD_DATA_PORT->MODER&=~DATA_MODER_MODERx;						//Input mode PB6 PB7
	KBD_DATA_PORT->OTYPER&=~DATA_OTYPER_OTx;						//Push-pull
	KBD_DATA_PORT->OSPEEDR&=~DATA_OSPEEDER_OSPEEDRx;		// Low speed
	KBD_DATA_PORT->PUPDR&=~DATA_PUPDR_PUPDRx;						//No-pull Выключам подтяжку
	
	KBD_CLOCK_PORT->AFR&=~GPIO_AFIO;											// AF2		
	KBD_CLOCK_PORT->MODER&=~CLOCK_MODER_MODERx;						//Input mode PB6 PB7
	KBD_CLOCK_PORT->OTYPER&=~CLOCK_OTYPER_OTx;						//Push-pull
	KBD_CLOCK_PORT->OSPEEDR&=~CLOCK_OSPEEDER_OSPEEDRx;		// Low speed
	KBD_CLOCK_PORT->PUPDR&=~CLOCK_PUPDR_PUPDRx;						//No-pull Выключам подтяжку
	
}
/*
*/

/*
*/
ErrorStatus PS2_KBD_Timeout_TX ( volatile uint32_t nCount ){
	
		ErrorStatus error = ERROR;
		while(nCount--)
		{
			if(PS2_KBD_VAR.mode==RX_MODE)
			{
				error=SUCCESS;
				break;
			}
		}
		return error;

}
/*
*/
ErrorStatus PS2_KBD_Timeout_RX ( volatile uint32_t nCount ){

	ErrorStatus error = ERROR;
	PS2_KBD_VAR.buff[0]=0;
	while ( nCount -- )
	{
		if ( PS2_KBD_VAR.buff[0]!=0 )
		{
			error = SUCCESS;
			break;
		} 
	} 
	return error;
}

/*
*/
ErrorStatus PS2_KBD_Tx(uint8_t cmd){
	
	uint16_t i;
	
	parity=0;
	PS2_KBD_VAR.send_code=cmd;
			
	for(i=0;i<8;i++)									//
	{																	//
		if(cmd&0x01)										//	Вычислим бит паритета таким примитивным способом.	
			parity++;											//	
		cmd>>=1;												//
	}
	if(parity%2)											//
		parity=0x0;											//
	else
		parity=0x01;										//
	
	
	//TIM->DIER&=~TIM_DIER_TIE;							//Запрет прерывания от TIM на время дерганья ножкой clock
	KBD_CLOCK_PORT->MODER&=~CLOCK_MODER_MODERx_1;
	KBD_CLOCK_PORT->MODER|=CLOCK_MODER_MODERx;  			//General purpose output mode
	
	KBD_CLOCK_PORT->BSRR=CLOCK_BSRR_BR;				//clock Low
	
	for(i=0;i<4000;i++);								// 20000 states (111 uSec)
	TIMx->CNT=0;												// Очень важно обнулять CNT перед отправкой след. команды. Не могу объяснить
																			// экспериментально вычислил.
	KBD_DATA_PORT->BSRR=DATA_BSRR_BR;				//data low
		
	KBD_CLOCK_PORT->BSRR=CLOCK_BSRR_BS;			//clock high
	
	
	KBD_CLOCK_PORT->MODER&=~CLOCK_MODER_MODERx;
	KBD_CLOCK_PORT->MODER|=CLOCK_MODER_MODERx_1; 				//Alternate function mode	
	//TIM->DIER|=TIM_DIER_TIE;							 // Разрешаем вновь прерывания для TIM
	
	PS2_KBD_VAR.receive_code=0;
	PS2_KBD_VAR.mode=TX_MODE;	
	
	if(PS2_KBD_Timeout_TX(20000)==ERROR)					
		return ERROR;														
	if(PS2_KBD_Timeout_RX(20000)==ERROR)
			return ERROR;
	if(PS2_KBD_VAR.buff[0]!=PS2_KBD_CMD_ACK)
		return ERROR;
	return SUCCESS;
}
/*
*/

/*
*/
uint8_t PS2_KBD_CheckScancode(uint8_t scan){
	static volatile uint8_t count_byte=0,last_byte=0;	

	if(last_byte)
	{
		//PS2_KBD_VAR.buff[count_byte]=scan;
		last_byte=0;
		count_byte=0;
		if((scan==SHIFT_L)||(scan==SHIFT_R))
			ControlKey.shift_press=0;
		else if(scan==ALT)
			ControlKey.alt_press=0;
		else if(scan==CTRL)
			ControlKey.ctrl_press=0;
		
		//PS2_KBD_VAR.status=KEY_RELEASED;
		return 0;
	}
	PS2_KBD_VAR.buff[count_byte]=scan;
	count_byte++;
	
	if(scan==0xF0)
	{	last_byte=1;
			return 0;
	}
	if(scan==0xE0)	
		PS2_KBD_VAR.extkey=1;
	else
	{	count_byte=0;
		PS2_KBD_VAR.status=KEY_PRESSED;
	}
	scancode=0;
	return 0;
}

/*
*/
void PS2_KBD_ISR(void){
	uint8_t temp;
	
	temp=TIMx->CNT;
	
	if(PS2_KBD_VAR.mode==RX_MODE)//if(PS2_MOUSE.status==MOUSE_AKTIV)
	{
		if((temp>1)&&(temp<10))
		{
			scancode>>=1;
			if(KBD_DATA_PORT->IDR&DATA_IDR_IDRx) 
				scancode|=0x80;
		}
	}
	else if(PS2_KBD_VAR.mode==TX_MODE)
	{
		
		if(temp==1)
		{
		scancode=PS2_KBD_VAR.send_code;	
		if(scancode&0x01)
			KBD_DATA_PORT->BSRR=DATA_BSRR_BS;
		else
			KBD_DATA_PORT->BSRR=DATA_BSRR_BR;
		scancode>>=1;
		}
		else if((temp>1)&&(temp<9))
		{
			if(scancode&0x01)
				KBD_DATA_PORT->BSRR=DATA_BSRR_BS;
			else
				KBD_DATA_PORT->BSRR=DATA_BSRR_BR;
			scancode>>=1;
		}
		else if(temp==9)
		{
			if(parity)
				KBD_DATA_PORT->BSRR=DATA_BSRR_BS;
			else
				KBD_DATA_PORT->BSRR=DATA_BSRR_BR;
		
		}
		else if(temp==10)
		{
			KBD_DATA_PORT->BSRR=DATA_BSRR_BS;					//Stop bit
		}
	}
}
/*
*/
