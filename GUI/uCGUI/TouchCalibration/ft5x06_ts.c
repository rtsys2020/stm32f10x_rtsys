/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : i2c_ee.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and I2C M24C08 EEPROM.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "ft5x06_ts.h"
//#include "MMA8452Q.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : FT5x06_GPIO_Config
* Description    : Configure the used I/O ports pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FT5x06_GPIO_Config(void)
{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//
//  /* Configure I2C1 pins: SCL and SDA */
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
}

#if 1


/*****************************************************************************
 �� �� ��  : FT5x06_Start
 ��������  : IIC�����ź�
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
void FT5x06_Start (void)
{
//	FT5x06_SDA=HIGH;
//	FT5x06_SCL=HIGH;
//	delay_asm1(6*2);
//	FT5x06_SDA=LOW;
//	delay_asm1(6*2);
//	FT5x06_SCL=LOW;
//	delay_asm1(6);
	
}
/*****************************************************************************
 �� �� ��  : FT5x06_Stop
 ��������  : IIC�����ź�
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
void FT5x06_Stop (void)
{
//	FT5x06_SDA=LOW;
//	FT5x06_SCL=LOW;
//	delay_asm1(6*2);
//	FT5x06_SCL=HIGH;
//	delay_asm1(6*2);
//	FT5x06_SDA=HIGH;
//	delay_asm1(6*4);
}

/*****************************************************************************
 �� �� ��  : FT5x06_WriteByte
 ��������  : IICд��һ���ֽ�
 �������  : u8 txByte--> ��д����ֽ�
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8_t FT5x06_WriteByte (uint8_t txByte)
{
//    u8 mask,erro=0;
//    for (mask=0x80; mask>0; mask>>=1)
//    {
//	    if ((mask & txByte) == 0) FT5x06_SDA=LOW;
//		else FT5x06_SDA=HIGH;
//		delay_asm1(6);
//		FT5x06_SCL=HIGH;
//		delay_asm1(6*3);
//		FT5x06_SCL=LOW;
//		delay_asm1(6*3);
//    }
//    FT5x06_SDA=HIGH; //release SDA-line
//    FT5x06_SCL=HIGH; //clk #9 for ack
//    delay_asm1(6*3);
//	if(READ_SDA==HIGH) erro=0;
//    FT5x06_SCL=LOW;
//	delay_asm1(6*3);
    return 0; //return error code
}

/*****************************************************************************
 �� �� ��  : FT5x06_ReadByte
 ��������  : IIC��ȡһ���ֽ�
 �������  : etI2cAck ack  ACK������Ӧ�ӻ� NO_ACK ��������Ӧ�ӻ�
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8_t FT5x06_ReadByte (uint8_t ack)
{
  uint8_t mask,rxByte=0;
//	FT5x06_SDA=HIGH;
//	delay_asm1(6);
//	for (mask=0x80; mask>0; mask>>=1)
//	{
//	    FT5x06_SCL=HIGH; //start clock on SCL-line
//		delay_asm1(6*3);
//		if(READ_SDA==1) rxByte=(rxByte | mask);
//		FT5x06_SCL=LOW;
//		delay_asm1(6*3);
//	}
//	FT5x06_SDA=ack;
//	delay_asm1(6);
//	FT5x06_SCL=HIGH; //clk #9 for ack
//	delay_asm1(6*3);
//	FT5x06_SCL=LOW;
//	FT5x06_SDA=HIGH;
//	delay_asm1(6*3);
	return rxByte; 
}

/*****************************************************************************
 �� �� ��  : IIC_RegWrite
 ��������  : ����FT5x06��һ���Ĵ���
 �������  : u8 device_add  -->FT5x06�ĵ�ַ
             u8 reg         --> FT5x06�ļĴ�����ַ
             u8 val         --> д����Ӧ�Ĵ�����ֵ
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
void FT5x06_RegWrite(uint8_t reg,uint8_t val)
{
  FT5x06_Start();                                  
  FT5x06_WriteByte(FT5x06_ADD);                      
  FT5x06_WriteByte(reg);                         
  FT5x06_WriteByte(val);                         
  FT5x06_Stop();                               
}

/*****************************************************************************
 �� �� ��  : FT_IIC_RegRead
 ��������  : ��ȡFT5x06��һ���Ĵ�����ֵ
 �������  : u8 reg         -->FT5x06�ļĴ�����ַ
 �������  : ��
 �� �� ֵ  : ��ȡ�ļĴ�����ֵ
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8_t FT_IIC_RegRead(uint8_t reg)
{
  uint8_t b;
  FT5x06_Start();                                  
  FT5x06_WriteByte(FT5x06_ADD);                 
  FT5x06_WriteByte(reg);                        
  FT5x06_Start();                           
  FT5x06_WriteByte(FT5x06_ADD+1);                    
//  b = FT5x06_ReadByte(NO_ACK);
  FT5x06_Stop();                                 
  return b;
}


/*****************************************************************************
 �� �� ��  : IIC_RegWriteN
 ��������  : ����д��FT5x06�Ķ���Ĵ���
 �������  : u8 reg1        -->Ҫд�����ʼ�ļĴ�����ַ
             u8 N           -->Ҫд��ļĴ�������
             u8 *array      -->д��Ĵ���ֵ�洢λ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
void FT5x06_RegWriteN(uint8_t reg1,uint8_t N,uint8_t *array)
{                              
  FT5x06_Start();                                 
  FT5x06_WriteByte(FT5x06_ADD);                      
  FT5x06_WriteByte(reg1);                     
  while (N>0)                               
  {
    FT5x06_WriteByte(*array);
    array++;
    N--;
  }
  FT5x06_Stop();                                
}


/*****************************************************************************
 �� �� ��  : FT5x06_RegReadN
 ��������  : ������ȡFT5x06�Ķ���Ĵ�����ֵ
 �������  : u8 reg1        -->Ҫ����ʼ�ļĴ�����ַ
             u8 N           -->Ҫ����ʼ�ļĴ�������
             u8 *array      -->��ȡ�ļĴ���ֵ�洢λ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2013��2��25��
    ��    ��   : SiminLee
    �޸�����   : �����ɺ���

*****************************************************************************/
void FT5x06_RegReadN(uint8_t reg1, uint8_t N, uint8_t *array)
{
  uint8_t b;
  FT5x06_Start();                                  
  FT5x06_WriteByte(FT5x06_ADD);                      
  FT5x06_WriteByte(reg1);                        
  FT5x06_Start();                           
  FT5x06_WriteByte(FT5x06_ADD+1);                    
  //b = IIC_ReadByte(ACK);                        
  while (N>1)                                   
  {
//    b = FT5x06_ReadByte(ACK);
    *array = b;
    array++;
    N--;
  }
//  b = FT5x06_ReadByte(NO_ACK);
  *array = b;                                   
  FT5x06_Stop();                                
}
/*******************************************************************************
* Function Name  : FT5x06_identify
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t FT5x06_identify(void)
{
	int id,ver;
	id = FT_IIC_RegRead(FT5X0X_REG_CIPHER);
	if (id != FT5x06_CHIP_ID) {
//		printf("ID %d not supported\r\n", id);
		return 0;
	}
	ver = FT_IIC_RegRead(FT5X0X_REG_FIRMID);
	if (ver < 0) {
//		printf("could not read the firmware version\r\n");
		return 0;
	}

//	printf("FT5x06 firmware version %d\r\n", ver);

	return 1;

}
/*******************************************************************************
* Function Name  : FT5x06_interrupt
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FT5x06_GetData(int* x, int* y)
{

	static int Xposition_H,Xposition_L,Yposition_H,Yposition_L,Xposition,Yposition;

//	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
//	if(TOUCH_PRESSING == 0)
	{
		  Xposition_H=FT_IIC_RegRead(FT5X0X_REG_TOUCH1_XH);
		  Xposition_L=FT_IIC_RegRead(FT5X0X_REG_TOUCH1_XL);
		  Xposition = (Xposition_H&0x0f)*256+Xposition_L;
		  Yposition_H=FT_IIC_RegRead(FT5X0X_REG_TOUCH1_YH);
		  Yposition_L=FT_IIC_RegRead(FT5X0X_REG_TOUCH1_YL);
		  Yposition = (Yposition_H&0x0f)*256+Yposition_L;

		  *x = Xposition;
		  *y = 480 - Yposition;

		  //printf("Xposition= [%d],Yposition = [%d]\r\n", Xposition,Yposition);
		    /* Clear the EXTI Line 4 */
//		  EXTI_ClearITPendingBit(EXTI_Line4);
  	}	
}
/*******************************************************************************
* Function Name  : FT5x06_init
* Description    : Init FT5x06
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FT5x06_init(void)
{
	FT5x06_GPIO_Config();
//	printf("[FTS] ft5x0x_ts_probe, driver version is %s.\r\n", CFG_FTS_CTP_DRIVER_VERSION);
	if(!FT5x06_identify())
	{
//		printf("[FTS] ==identify failed =\r\n");
	}
	else
	{
//		printf("[FTS] ==init over =\r\n");
	}
}

#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
