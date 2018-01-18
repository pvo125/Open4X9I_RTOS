#ifndef _PS2_Keyboard_H_
#define _PS2_Keyboard_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"


#define 	CLOCK_PIN								7				
#if (CLOCK_PIN>7)
	#define AFR	 AFR[1]	
#else
	#define AFR  AFR[0]	
#endif

#define GPIO_AFIO									(uint32_t)(0x20000000)
#define __TIMx_CLK_ENABLE()				__TIM3_CLK_ENABLE()
#define __TIMx_CLK_DISABLE()			__TIM3_CLK_DISABLE()
#define TIMx_IRQn									TIM3_IRQn
#define TIMx											TIM3
/**************************************************************************/
#define DATA_BSRR_BR							GPIO_BSRR_BR_1
#define DATA_BSRR_BS							GPIO_BSRR_BS_1
#define DATA_IDR_IDRx							GPIO_IDR_IDR_1

#define DATA_ODR_ODRx							GPIO_ODR_ODR_1
#define DATA_MODER_MODERx					GPIO_MODER_MODER1_0
#define DATA_OTYPER_OTx						GPIO_OTYPER_OT_1
#define DATA_OSPEEDER_OSPEEDRx		GPIO_OSPEEDER_OSPEEDR1_0	
#define DATA_PUPDR_PUPDRx					GPIO_PUPDR_PUPDR1_0
/************************************************************************/
#define CLOCK_BSRR_BR							GPIO_BSRR_BR_7
#define CLOCK_BSRR_BS							GPIO_BSRR_BS_7

#define CLOCK_ODR_ODRx						GPIO_ODR_ODR_7
#define CLOCK_MODER_MODERx				GPIO_MODER_MODER7_0
#define CLOCK_MODER_MODERx_1			GPIO_MODER_MODER7_1
#define CLOCK_OTYPER_OTx					GPIO_OTYPER_OT_7
#define CLOCK_OSPEEDER_OSPEEDRx		GPIO_OSPEEDER_OSPEEDR7_0	
#define CLOCK_PUPDR_PUPDRx				GPIO_PUPDR_PUPDR7_0
/**************************************************************************/
#define KBD_DATA_PORT						GPIOC
#define KBD_CLOCK_PORT					GPIOA

/***************************************************************************/

#define TX_MODE									1
#define RX_MODE									0

/***********************************     Keyboard     ******************************************************************/
#define  KEYBOARD_EXTEND_CODE 					0xE0
#define  KEYBOARD_BREAK_CODE 						0xF0
#define  PS2_KBD_CMD_ACK       					0xFA  // ACK

#define SET_KEYBOARD_INDICATORS 			 	0xED

#define KEYBOARD_CAPS_LOCK_ON     			4
#define KEYBOARD_NUM_LOCK_ON      			2
#define KEYBOARD_SCROLL_LOCK_ON   			1

typedef enum {
  KEY_RELEASED =0,      // 
  KEY_PRESSED           // 
}PS2_KBD_BTN;
/*
*/
typedef struct {
  uint8_t mode;							// RX-TX mode
	PS2_KBD_BTN status; 			//
	uint8_t receive_code;   	// 
  uint8_t send_code;      	// 
  uint8_t extkey;
	uint8_t buff[5];          // 
}PS2_KBD_VAR_t;

typedef struct{
	char scancode;
	char latCapital;
	char latLower;
	char cirCapital;
	char cirLower;
}KeyTypeDef;

typedef struct{
	uint8_t shift_press;
	uint8_t alt_press;
	uint8_t capslock_on;
	uint8_t ctrl_press;
	uint8_t numlock_on;
	uint8_t EngRus;
}ControlKeyTypeDef;

#define	ENTER 			0x5A
#define	BACKSPACE 	0x66
#define	SPACE 			0x29
#define	TAB					0x0D
#define	CAPSLOCK		0x58
#define	NUMLOCK			0x77
#define	ALT				 	0x11
#define	SHIFT_L		 	0x12
#define	SHIFT_R		 	0x59
#define	CTRL			 	0x14

#define DIV					0x4A




void PS2_KBD_LowLevelInit(void);
void PS2_KBD_LowLevelDeInit(void);

ErrorStatus PS2_KBD_Timeout_TX ( volatile uint32_t nCount );
ErrorStatus PS2_KBD_Timeout_RX ( volatile uint32_t nCount );
ErrorStatus PS2_KBD_Tx(uint8_t cmd);

uint8_t PS2_KBD_CheckScancode(uint8_t scan);
void PS2_KBD_ISR(void);

/****************************************************/
#endif
