
#include "lcd_general.h"

#define CH_X  0xD0//0x90
#define CH_Y  0x90//0xd0

/*
for stmfans-ev board
lcd:
PD0 -> nCS  PD1 -> RS  PD2 -> nWR  PD3 -> nRD  PD4 -> nRESET
touch:
PD5 -> TS_CLK  PD6 -> TS_nCS  PD7 -> TS_DIN  PD8 -> TS_BUSY  PD9 -> TS_nPENIRQ  PD10 -> TS_DOUT
*/





void Touch_Start(void)
{

}

void Touch_Write(u8 d)
{
	u8 buf, i ;
	

}

u16  Touch_Read(void)
{
	u16 buf ;
	u8 i ;


	return( buf ) ;
}

u8  Touch_Busy(void)
{

  return 1;
}

u8  Touch_PenIRQ(void)
{

  return 1;
}

void Touch_Initializtion()
{

}

u16 _AD2X(int adx)
{
  u16 sx=0;
  int r = adx - 280;
  r *= 239;
  sx=r / (3740 - 280);
  if (sx<=0 || sx>240)
    return 0;
  return sx;
}

u16 _AD2Y(int ady)
{
  u16 sy=0;
  int r = ady - 230;
  r *= 319;
  sy=r/(3720 - 230);
  if (sy<=0 || sy>320)
    return 0;
  return sy;
}

u16  Touch_MeasurementX(void)
{
  u8 i;
  u16 p=0;
  for (i=0;i<8;i++)
  {
    p+=Touch_GetPhyX();
    //    SPI_delay(50);
  }
  p>>=3;
  
  return ( p );
}

u16  Touch_MeasurementY(void)
{
  u8 i;
  u16 p=0;
  for (i=0;i<8;i++)
  {
    p+=Touch_GetPhyY();
//    SPI_delay(50);
  }
  p>>=3;
  
  return ( p );
}

u16  Touch_GetPhyX(void)
{
  if (Touch_PenIRQ()) return 0;
  Touch_Start();
  Touch_Write(0x00);
  Touch_Write(CH_X);
  while(!Touch_Busy());
  return (Touch_Read());
}

u16  Touch_GetPhyY(void)
{
  if (Touch_PenIRQ()) return 0;
  Touch_Start();
  Touch_Write(0x00);
  Touch_Write(CH_Y);
  while(!Touch_Busy());
  return (Touch_Read());
}
/*
u16 Dx(u16 xx)
{
  return (_AD2X(xx));
}
u16 Dy(u16 yy)
{
  return (_AD2Y(yy));
}
*/
