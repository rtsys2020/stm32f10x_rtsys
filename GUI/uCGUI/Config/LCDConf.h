#ifndef LCDCONF_H
#define LCDCONF_H


#define LCD_XSIZE          (160)
#define LCD_YSIZE          (128)
#define LCD_CONTROLLER     (9320)
#define LCD_BITSPERPIXEL   (16)
#define LCD_FIXEDPALETTE   (565)
#define LCD_SWAP_RB        (0)
extern 	void Lcd_Init(void); 
#define LCD_INIT_CONTROLLER() Lcd_Init();   //
#endif /* LCDCONF_H */

