/**************************************************************
** 	������
**	���ܽ��ܣ� ��������������
**  �汾��V1.0  
**	��̳��www.openmcu.com
**	�Ա���http://shop36995246.taobao.com/   
**  ����֧��Ⱥ��121939788 
***************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "../TouchCalibration/Touch.h"

#include "stm32f10x.h"
//#include "SSD1963.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
Pen_Holder Pen_Point;	/* �����ʵ�� */ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//EXTI_InitTypeDef  EXTI_InitStructure;

uint8_t TOUCH_PRESSING;
TTOUCH_P g_tTP;

#if defined TOUCH_SPI_SIM
void Delay1(unsigned int x)
{
	unsigned int y = 0;
	for(;x>0;x--)
	for(y = 0;y<10;y++);
}
//SPIд����
//��7843д��1byte����   
void ADS_Write_Byte(u8 num)    
{  
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TDIN=1;  
		else TDIN=0;   
		num<<=1;    
		TCLK=0;//��������Ч	   	 
		TCLK=1;      
	} 			    
} 		 
//SPI������ 
//��7846/7843/XPT2046/UH7843/UH7846��ȡadcֵ	   
u16 ADS_Read_AD(u8 CMD)	  
{ 	 
	u8 i;
	u8 count=0; 	  
	u16 Num=0; 
	TCLK=0;//������ʱ�� 	 
	TCS=0; //ѡ��ADS7843	 
	ADS_Write_Byte(CMD);//����������
	for(i=100;i>0;i--);
//	Delay1_us(6);//ADS7846��ת��ʱ���Ϊ6us
	TCLK=1;//��1��ʱ�ӣ����BUSY   	    
	TCLK=0; 	 
	for(count=0;count<16;count++)  
	{ 				  
		Num<<=1; 	 
		TCLK=0;//�½�����Ч  	    	   
		TCLK=1;
		if(DOUT)Num++; 		 
	}  	
	Num>>=4;   //ֻ�и�12λ��Ч.
	TCS=1;//�ͷ�ADS7843	 
	return(Num);   
}
//��ȡһ������ֵ
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
#define READ_TIMES 15 //��ȡ����
#define LOST_VAL 5	  //����ֵ
u16 ADS_Read_XY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)
	{				 
		buf[i]=ADS_Read_AD(xy);	    
	}				    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 
//���˲��������ȡ
//��Сֵ��������100.
u8 Read_ADS(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=ADS_Read_XY(CMD_RDY);
	ytemp=ADS_Read_XY(CMD_RDX);	  				
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}
//2�ζ�ȡADS7846,������ȡ2����Ч��ADֵ,�������ε�ƫ��ܳ���
//50,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
#define ERR_RANGE 50 //��Χ 
u8 Read_ADS2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=Read_ADS(&x1,&y1);   
    if(flag==0)return(0);
    flag=Read_ADS(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
} 
//��ȡһ������ֵ	
//������ȡһ��,֪��PEN�ɿ��ŷ���!					   
u8 Read_TP_Once(void)
{
	unsigned  char t = 0;  
	Pen_Int_Set(0);//�ر��ж�
	Pen_Point.Key_Sta=Key_Up;		  
		Read_ADS2(&Pen_Point.X,&Pen_Point.Y);
		Pen_Point.X = 0,Pen_Point.Y = 0;
	while((Pen_Point.X == 1730) || (Pen_Point.Y == 2428) || (Pen_Point.Y == 0) ||(Pen_Point.Y == 1730))
	{											  
		Delay1(8);
		Read_ADS2(&Pen_Point.X,&Pen_Point.Y); 
	}
	while(PEN==0&&t<=250)
	{
		t++;
		Delay1(3);
	};
	Pen_Int_Set(1);//�����ж�		 
	if(t>=250)return 0;//����2.5s ��Ϊ��Ч	x = 1730 ,y = 2428
	else return 1;	
}

#endif
/*****************************************************************************
** ��������: Touch_Configuration
** ��������: ������IO����
** ��  ����: Dream
** �ա�  ��: 2010��12��06��
*****************************************************************************/

void Touch_Configuration()	//bsp.c������
{	
 #if 0
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO
							, ENABLE );  //��Ҫ����

	//������SPI���GPIO��ʼ��	sck mosi
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //ͨ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	//������SPI���GPIO��ʼ��	miso
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	//Configure PB12 pin: TP_CS pin 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//ͨ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
//sd_cs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//ͨ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);

//flash_cs 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//ͨ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);

	GPIO_SetBits(GPIOG,GPIO_Pin_15);     //��ѡ��SD��
    GPIO_SetBits(GPIOG,GPIO_Pin_11);     //��ѡ��FLASH��

	//Configure PG7 pin: TP_INT pin 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	//��������
	GPIO_Init(GPIOG,&GPIO_InitStructure);


	/*********gan**********/
	{
		SPI_InitTypeDef   SPI_InitStructure; 
   		//SPI2 Periph clock enable 
  		RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

		// SPI2 Config  
  		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
	  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
	  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
	  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   
	  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; 
	  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	  	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	  	SPI_Init(SPI2,&SPI_InitStructure); 

  		// SPI2 enable  
  		SPI_Cmd(SPI2,ENABLE);
	}
	#endif
}
/*****************************************************************************
** ��������: Pen_Int_Set
** ��������: PEN�ж�����
				EN=1�������ж�
					EN=0: �ر��ж�
** ��  ����: Dream
** �ա�  ��: 2010��12��06��
*****************************************************************************/	 
void Pen_Int_Set(u8 en)
{
	if(en)EXTI->IMR|=1<<7;   //����line13�ϵ��ж�	  	
	else EXTI->IMR&=~(1<<7); //�ر�line13�ϵ��ж�	   
}
/*****************************************************************************
** ��������: EXTI15_10_IRQHandler
** ��������: �жϴ�����
				�ж�,��⵽PEN�ŵ�һ���½���.
					��λPen_Point.Key_StaΪ����״̬
						�ж���4���ϵ��жϼ��
** ��  ����: Dream
** �ա�  ��: 2010��12��06��
*****************************************************************************/
/*void EXTI15_10_IRQHandler()
{
	u16 i;
  	if(NVIC_GetIRQChannelPendingBitStatus(EXTI_IMR_MR13) != RESET)
	{
    	NVIC_ClearIRQChannelPendingBit(EXTI_IMR_MR13);	 //����жϹ���λ
		for(i=1000;i>0;i--); 			//��ʱȥ����
		Pen_Point.Key_Sta=Key_Down;//�������� 		 		  				 
	}
}*/
#ifdef ADJ_SAVE_ENABLE
#define SAVE_ADDR_BASE 40
/*****************************************************************************
** ��������: Save_Adjdata
** ��������: ����У׼������EEPROAM�е� ��ַ40
				�˲����漰��ʹ���ⲿEEPROM,���û���ⲿEEPROM,���δ˲��ּ���
				������EEPROM����ĵ�ַ�����ַ,ռ��13���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
** ��  ����: Dream
** �ա�  ��: 2010��12��06��
*****************************************************************************/
void Save_Adjdata(void)
{
	u8 temp[4];
	//����У�����!		   							  
	temp[0]=(u32)(Pen_Point.xfac*100000000)&0xff;	//����xУ������    
	temp[1]=(u32)(Pen_Point.xfac*100000000)>>8&0xff;//����xУ������    
	temp[2]=(u32)(Pen_Point.xfac*100000000)>>16&0xff;//����xУ������        
	temp[3]=(u32)(Pen_Point.xfac*100000000)>>24&0xff;//����xУ������        
   	I2C_Write(&temp[0],SAVE_ADDR_BASE,4);

	temp[0]=(u32)(Pen_Point.yfac*100000000)&0xff;//����xУ������    
	temp[1]=(u32)(Pen_Point.yfac*100000000)>>8&0xff;//����xУ������    
	temp[2]=(u32)(Pen_Point.yfac*100000000)>>16&0xff;//����xУ������        
	temp[3]=(u32)(Pen_Point.yfac*100000000)>>24&0xff;//����xУ������ 

   	I2C_Write(&temp[0],SAVE_ADDR_BASE+4,4);
	temp[0]=(u32)(Pen_Point.xoff*100000000)&0xff;//����xУ������    
	temp[1]=(u32)(Pen_Point.xoff*100000000)>>8&0xff;//����xУ������    
   	I2C_Write(&temp[0],SAVE_ADDR_BASE+8,2);
	//����xƫ����
	//����yƫ����
	temp[0]=(u32)(Pen_Point.yoff*100000000)&0xff;//����xУ������    
	temp[1]=(u32)(Pen_Point.yoff*100000000)>>8&0xff;//����xУ������    
   	I2C_Write(&temp[0],SAVE_ADDR_BASE+10,2);
	I2C_Read(&temp[0],SAVE_ADDR_BASE+12,1);
	temp[0]&=0XF0;
	temp[0]|=0X0A;//���У׼����
	I2C_Write(&temp[0],SAVE_ADDR_BASE+12,1);			 
}
/*****************************************************************************
** ��������: Get_Adjdata
** ��������: �õ�������EEPROM�����У׼ֵ
				����ֵ��1���ɹ���ȡ����
						 0����ȡʧ�ܣ�Ҫ����У׼
** ��  ����: Dream
** �ա�  ��: 2010��12��06��
*****************************************************************************/       
u8 Get_Adjdata(void)
{	
	u8 temp[4];
	u32 tempfac=0;
	I2C_Read(&temp[0],52,1); //����ʮ���ֽڵĵ���λ��������Ƿ�У׼���� 		 
	if((temp[0]&0X0F)==0X0A)		//�������Ѿ�У׼����			   
	{   
		I2C_Read(&temp[0],40,4);
		tempfac = temp[0]|(temp[1]<<8)|(temp[2]<<16)|(temp[3]<<24);	   
		Pen_Point.xfac=(float)tempfac/100000000;//�õ�xУ׼����
		I2C_Read(&temp[0],44,4);
		tempfac = temp[0]|(temp[1]<<8)|(temp[2]<<16)|(temp[3]<<24);	   
			          
		Pen_Point.yfac=(float)tempfac/100000000;//�õ�yУ׼����
	    //�õ�xƫ����
		I2C_Read(&temp[0],48,2);		
		tempfac = temp[0]|(temp[1]<<8);	   		   	  
		Pen_Point.xoff=tempfac;					 
	    //�õ�yƫ����
		I2C_Read(&temp[0],50,2);
		tempfac = temp[0]|(temp[1]<<8);	   
						 	  
		Pen_Point.yoff=tempfac;					 
		return 1;	 
	}
	return 0;
}
#endif

/******gan*********/

#define TP_CS()  //GPIO_ResetBits(GPIOC,GPIO_Pin_3)
#define TP_DCS() //GPIO_SetBits(GPIOC,GPIO_Pin_3)

unsigned char SPI_WriteByte(unsigned char data) 
{ 
 unsigned char Data01 = 0; 

   //Wait until the transmit buffer is empty 
//  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
  // Send the byte  
  SPI_I2S_ReceiveData(SPI1);
  SPI_I2S_SendData(SPI1,data); 

   //Wait until a data is received 
//  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
  // Get the received data 
  Data01 = SPI_I2S_ReceiveData(SPI1); 

  // Return the shifted data 
  return Data01; 
}  


void SpiDelay(unsigned int DelayCnt)
{
 unsigned int i;
 for(i=0;i<DelayCnt;i++);
}

u16 TPReadX(void)
{ 
   u16 x=0;
   TP_CS();
   SpiDelay(10);
   SPI_WriteByte(0xd0);
   SpiDelay(10);
   x=SPI_WriteByte(0x00);
   x<<=8;
   x+=SPI_WriteByte(0x00);
   SpiDelay(20);
   TP_DCS();
   //x = 3671 - x; 
   x = x>>3;

  GUI_DispDecAt(x, 300, 300, 5);
  return (x);
}

u16 TPReadY(void)
{
 u16 y=0;
  TP_CS();
  SpiDelay(10);
  SPI_WriteByte(0x90);
  SpiDelay(10);
  y=SPI_WriteByte(0x00);
  y<<=8;
  y+=SPI_WriteByte(0x00);
  SpiDelay(20);
  TP_DCS();
  //y = 3601 - y;
  y = y>>3;
   
  GUI_DispDecAt(y, 350, 300, 5);

  return (y);
}


unsigned int get_calibrate_state(void)
{
   return  1;

}

void get_calibrate_data(uint16_t* cal_data)
{
	int i;
	for(i=0;i<5;i++)
	{
	cal_data[i]= 0;
    } 
}

int  save_calibrate_to_flash(uint16_t* data)
{

    return 1;
}



//
//xy 1-- x, 2-- y
//
uint16_t TOUCH_DataFilter(uint8_t xy)
{
	uint16_t i, j;
    uint16_t buf[ADC_READ_TIME];
    uint16_t usSum;
    uint16_t usTemp;
	usSum = 0;
	
    for(i=0; i < ADC_READ_TIME; i++)
    {
     	if(xy == 1)
		{
        buf[i] = TPReadX();
		}else{
		buf[i] = TPReadY();
		}
    }
                        
    for(i = 0; i < ADC_READ_TIME - 1; i++)
    {
        for(j = i + 1; j < ADC_READ_TIME; j++)
        {
            if(buf[i] > buf[j])
            {
                usTemp = buf[i];
                buf[i] = buf[j];
                buf[j] = usTemp;
            }
        }
    }     
    
 
    for(i = DISCARD_No; i < ADC_READ_TIME - DISCARD_No; i++)
    {
        usSum += buf[i];
    }
   
    usTemp = usSum / (ADC_READ_TIME - 2 * DISCARD_No);

    return usTemp; 
}

 
uint8_t TOUCH_ReadAdcXY(uint16_t *_usX, uint16_t *_usY) 
{
    uint16_t iX1, iY1;
    uint16_t iX2, iY2;
    uint16_t iX, iY;

    iY1 = TOUCH_DataFilter(ADC_CH_Y);
    iX1 = TOUCH_DataFilter(ADC_CH_X);
    iY2 = TOUCH_DataFilter(ADC_CH_Y);
    iX2 = TOUCH_DataFilter(ADC_CH_X);

    iX = abs(iX1 - iX2);
    iY = abs(iY1 - iY2); 

    /* ǰ�����β�����+-ADC_ERR_RANGE�� */  
    if ((iX <= ADC_ERR_RANGE) && (iY <= ADC_ERR_RANGE))
    {       
        *_usX = (iX1 + iX2) / 2;
        *_usY = (iY1 + iY2) / 2;

        return 1;
    }
    else 
    {
        return 0;
    }      
} 



void TOUCH_Scan(void)
{

    uint8_t s_invalid_count = 0;
	uint8_t flag;
	
   
    if (TOUCH_PRESSING == 0)
    {   
	              
        while(!TOUCH_ReadAdcXY(&g_tTP.usAdcNowX, &g_tTP.usAdcNowY)&&(s_invalid_count<5));
        {
            s_invalid_count++;
        }
        
        if(s_invalid_count >= 5)
        {
            g_tTP.usAdcNowX = 0;
            g_tTP.usAdcNowY = 0;    
        }
    }
    else
    {
        g_tTP.usAdcNowX = 0;
        g_tTP.usAdcNowY = 0;
    }
	GUI_DispDecAt( g_tTP.usAdcNowX, 400, 400, 5);
	GUI_DispDecAt( g_tTP.usAdcNowY, 450, 450, 5);
}




/*********************************************************************************************************
** End of File
*********************************************************************************************************/
