#ifndef __LCDDRIVER_H
#define __LCDDRIVER_H

#include "stm32f4xx.h" 

#define LCD_XSIZE	480 
#define LCD_YSIZE 544

typedef struct{
	uint8_t XSize;
  uint8_t XDist;
  uint8_t BytesPerLine;
  const unsigned char * pData;
}GUI_CHARINFO;




extern uint16_t LCD_PutChar(char Char,uint16_t XPix_offset,uint8_t YLine_offset,uint8_t LayerIdx,uint32_t color);
extern uint16_t LCD_PutString(const char * pString,uint16_t XPix_offset,uint16_t YLine_offset,uint8_t LayerIdx,uint32_t color,uint8_t alpha);
extern void LCD_ClearRec(uint16_t XPos_offset,uint16_t YLine_offset,uint16_t XSize,uint16_t YSize,uint8_t IdxLayer,uint32_t color,uint8_t alpha);







#endif
