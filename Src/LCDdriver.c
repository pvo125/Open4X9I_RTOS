#include "stm32f4xx.h"                  // Device header
#include "LCDdriver.h"
#include "mytask.h"
extern const GUI_CHARINFO GUI_FontArialNarrow22_CharInfo[];

extern DMA2D_HandleTypeDef hdma2d;
extern LTDC_HandleTypeDef hltdc;

#define LCD_LAYER0_FRAME_BUFFER  ((uint32_t)0xD0000000)
#define LCD_LAYER1_FRAME_BUFFER  ((uint32_t)0xD00ff000)



const uint8_t _aMirror[] = {
  0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0,
  0x01, 0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1,
  0x02, 0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2, 0xB2, 0xC2, 0xD2, 0xE2, 0xF2,
  0x03, 0x13, 0x23, 0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3, 0xB3, 0xC3, 0xD3, 0xE3, 0xF3,
  0x04, 0x14, 0x24, 0x34, 0x44, 0x54, 0x64, 0x74, 0x84, 0x94, 0xA4, 0xB4, 0xC4, 0xD4, 0xE4, 0xF4,
  0x05, 0x15, 0x25, 0x35, 0x45, 0x55, 0x65, 0x75, 0x85, 0x95, 0xA5, 0xB5, 0xC5, 0xD5, 0xE5, 0xF5,
  0x06, 0x16, 0x26, 0x36, 0x46, 0x56, 0x66, 0x76, 0x86, 0x96, 0xA6, 0xB6, 0xC6, 0xD6, 0xE6, 0xF6,
  0x07, 0x17, 0x27, 0x37, 0x47, 0x57, 0x67, 0x77, 0x87, 0x97, 0xA7, 0xB7, 0xC7, 0xD7, 0xE7, 0xF7,
  0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78, 0x88, 0x98, 0xA8, 0xB8, 0xC8, 0xD8, 0xE8, 0xF8,
  0x09, 0x19, 0x29, 0x39, 0x49, 0x59, 0x69, 0x79, 0x89, 0x99, 0xA9, 0xB9, 0xC9, 0xD9, 0xE9, 0xF9,
  0x0A, 0x1A, 0x2A, 0x3A, 0x4A, 0x5A, 0x6A, 0x7A, 0x8A, 0x9A, 0xAA, 0xBA, 0xCA, 0xDA, 0xEA, 0xFA,
  0x0B, 0x1B, 0x2B, 0x3B, 0x4B, 0x5B, 0x6B, 0x7B, 0x8B, 0x9B, 0xAB, 0xBB, 0xCB, 0xDB, 0xEB, 0xFB,
  0x0C, 0x1C, 0x2C, 0x3C, 0x4C, 0x5C, 0x6C, 0x7C, 0x8C, 0x9C, 0xAC, 0xBC, 0xCC, 0xDC, 0xEC, 0xFC,
  0x0D, 0x1D, 0x2D, 0x3D, 0x4D, 0x5D, 0x6D, 0x7D, 0x8D, 0x9D, 0xAD, 0xBD, 0xCD, 0xDD, 0xED, 0xFD,
  0x0E, 0x1E, 0x2E, 0x3E, 0x4E, 0x5E, 0x6E, 0x7E, 0x8E, 0x9E, 0xAE, 0xBE, 0xCE, 0xDE, 0xEE, 0xFE,
  0x0F, 0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F, 0x8F, 0x9F, 0xAF, 0xBF, 0xCF, 0xDF, 0xEF, 0xFF,
};


uint8_t _swapBuffer[256];

/*
*   
*/

/*
*
*/


uint16_t LCD_PutChar(char Char,uint16_t XPix_offset,uint8_t YLine_offset,uint8_t LayerIdx,uint32_t color){
	uint8_t *pSrc,*pWR;
	uint8_t BytesPerLine;
	uint32_t dstAddr;
	uint8_t NumBytes;
		
	if(LayerIdx)
		dstAddr=LCD_LAYER1_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPix_offset)*4;  
	else
		dstAddr=LCD_LAYER0_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPix_offset)*4;
	
	if(Char < (char)0x80)
		Char-=0x1e;
	else
		Char-=0x3e;
	
	pSrc=(uint8_t*)GUI_FontArialNarrow22_CharInfo[Char].pData;
	BytesPerLine=GUI_FontArialNarrow22_CharInfo[Char].BytesPerLine;	
	NumBytes=BytesPerLine*22;
	pWR=_swapBuffer;
	
	osSemaphoreWait (xDMA2DSemaphoreHandle,100);
	do{
	*pWR++ = _aMirror[*pSrc++];
	} while (--NumBytes);	
	
	
	DMA2D->FGCOLR=color;
	DMA2D->OOR=LCD_XSIZE-BytesPerLine*2;
	HAL_DMA2D_Start_IT(&hdma2d,(uint32_t)_swapBuffer,dstAddr,BytesPerLine*2,22);
	
	dstAddr+=(GUI_FontArialNarrow22_CharInfo[Char].XDist+BytesPerLine/2)*4;
	
	return XPix_offset+GUI_FontArialNarrow22_CharInfo[Char].XDist+BytesPerLine/2;
}
/*
*
*/
uint16_t LCD_PutString(const char * pString,uint16_t XPix_offset,uint16_t YLine_offset,uint8_t LayerIdx,uint32_t color,uint8_t alpha){
	uint8_t *pSrc,*pWR;
	uint8_t BytesPerLine,temp,NumBytes;
	uint32_t dstAddr;
	
	if(pString==0)
		return XPix_offset;
	if(LayerIdx)
		dstAddr=LCD_LAYER1_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPix_offset)*4;  
	else
		dstAddr=LCD_LAYER0_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPix_offset)*4;
	while(*pString)
	{
		temp=*pString;
		if(temp<0x80)
			temp=*pString-0x1E;
		else
			temp=*pString-0x3e;
		pSrc=(uint8_t*)GUI_FontArialNarrow22_CharInfo[temp].pData;
		BytesPerLine=GUI_FontArialNarrow22_CharInfo[temp].BytesPerLine;	
		NumBytes=BytesPerLine*22;
		pWR=_swapBuffer;
		
		osSemaphoreWait (xDMA2DSemaphoreHandle,100);
		do{
			*pWR++ = _aMirror[*pSrc++];
		} while (--NumBytes);
		
		if(alpha!=0xFF)
		{
			DMA2D->FGPFCCR|=(alpha<<24);	
			DMA2D->FGPFCCR|=DMA2D_FGPFCCR_AM_1;
		}
		else
			DMA2D->FGPFCCR&=~DMA2D_FGPFCCR_AM_1;
			
		DMA2D->FGCOLR=color;
		DMA2D->OOR=LCD_XSIZE-BytesPerLine*2;
		HAL_DMA2D_Start_IT(&hdma2d,(uint32_t)_swapBuffer,dstAddr,BytesPerLine*2,22);
		pString++;
		XPix_offset+=(GUI_FontArialNarrow22_CharInfo[temp].XDist+BytesPerLine/2);
		dstAddr+=(GUI_FontArialNarrow22_CharInfo[temp].XDist+BytesPerLine/2)*4;
	}
		return XPix_offset; 
}

/*
*
*/
void LCD_ClearRec(uint16_t XPos_offset,uint16_t YLine_offset,uint16_t XSize,uint16_t YSize,uint8_t IdxLayer,uint32_t color,uint8_t alpha){
	
	osSemaphoreWait (xDMA2DSemaphoreHandle,100);
	
	DMA2D->CR      =  0x00030000UL;//|(1 << 9);       // Control Register (Register to memory and TCIE)
  DMA2D->OCOLR   = (alpha<<24)|color;                      // Output Color Register (Color to be used)
  if(IdxLayer)
		DMA2D->OMAR    = LCD_LAYER1_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPos_offset)*4;  // Output Memory Address Register (Destination address)
  else
		DMA2D->OMAR    = LCD_LAYER0_FRAME_BUFFER+(YLine_offset*LCD_XSIZE+XPos_offset)*4;
	DMA2D->OOR     = DMA2D->OOR=LCD_XSIZE-XSize;//LCD_XSIZE;                         // Output Offset Register (Destination line offset)
  //
  //DMA2D->OPFCCR  = PixelFormat;                     // Output PFC Control Register (Defines the output pixel format)
  DMA2D->NLR     = (uint32_t)((XSize << 16) |(uint16_t)YSize); // Number of Line Register (Size configuration of area to be transfered)
  __HAL_DMA2D_ENABLE_IT(&hdma2d, DMA2D_IT_TC|DMA2D_IT_TE|DMA2D_IT_CE);
	//DMA2D->CR     |= 1;                               // Control Register (Start operation)
  __HAL_DMA2D_ENABLE(&hdma2d);
		
}







