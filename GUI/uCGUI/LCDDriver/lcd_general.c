#include "stm32f10x.h"
#include "LCD_Config.h"
#include "ST7735.h"
#include <board.h>


void LCD_X_Init(void)
{
//  ST7735(PB12, PA10, PA8);
  ST7735(PB12, PA2, PA0);
  initR(INITR_BLACKTAB);
  invertDisplay(0);
  setRotation(1);

}



void GUI_Line(u16 x0, u16 y0, u16 x1, u16 y1,u16 color)
{
  drawLine(x0,y0,x1,y1,color);
}





unsigned int LCD_GetPoint(int x,int y)
{
  return 0x0000;
}



void LCD_Clear(u16 c)
{
//  fillRect(0, 0, 120, 160, c);
}


void LCD_SetPoint(u16 x,u16 y,u16 c)
{
  drawPixel(x,y,c);
}

void Lcd_Init(void)
{
  	LCD_X_Init();
}

