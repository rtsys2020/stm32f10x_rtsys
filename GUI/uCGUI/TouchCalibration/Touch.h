#ifndef __TOUCH_H
#define __TOUCH_H	

//#include "touchc.h"
#include <stdint.h>
#include "GUI.h"
#define ADC_READ_TIME			8
#define DISCARD_No				2
#define ADC_ERR_RANGE			12

#define ADC_CH_X				1
#define ADC_CH_Y				2

/* ADS7843/7846/UH7843/7846/XPT2046/TSC2046 ָ� */\
#define CMD_RDX	0XD0  //0B11010000���ò�ַ�ʽ��X���� 
#define CMD_RDY 0X90  //0B10010000���ò�ַ�ʽ��Y���� 
/* ʹ��24LC02 */
//#define ADJ_SAVE_ENABLE	    
 
/*������оƬ������������	*/   
#define PEN  PGin(7)   //PG7  INT
#define DOUT PBin(14)   //PB14  MISO
#define TDIN PBout(15)  //PB15  MOSI
#define TCLK PBout(13)  //PB13  SCLK
#define TCS  PBout(12)  //PB12  CS    

/* ����״̬	*/ 
#define Key_Down 0x01
#define Key_Up   0x00 
/*
typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;*/

#define CAL_STATE_ADDR //BANK511_WRITE_START_ADDR
#define CAL_DATA_ADDR  //BANK511_WRITE_START_ADDR+2
#define CAL_STATE_ADJ_READY 0x01
#define CAL_STATE_NOT_ADJ   0x00

typedef struct{

	uint16_t usAdcNowX;
	uint16_t usAdcNowY;

}TTOUCH_P;

/* �ʸ˽ṹ�� */
typedef struct 
{
	uint16_t X0;//ԭʼ����
	uint16_t Y0;
	uint16_t X; //����/�ݴ�����
	uint16_t Y;
	uint8_t  Key_Sta;//�ʵ�״̬
	//������У׼����
	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;




extern Pen_Holder Pen_Point; 
extern TTOUCH_P g_tTP;

void Touch_Init(void);
void Pen_Int_Set(uint8_t en);
void Touch_Configuration(void);
void ADS_Write_Byte(uint8_t num);
uint16_t ADS_Read_AD(uint8_t CMD);
uint16_t ADS_Read_XY(uint8_t xy);
uint8_t Read_TP_Once(void);
uint8_t Read_ADS2(uint16_t *x,uint16_t *y);
uint8_t Read_ADS(uint16_t *x,uint16_t *y);
void Drow_Touch_Point(uint16_t x,uint16_t y);
uint8_t Get_Adjdata(void);
void Refreshes_Screen(void);
void Convert_Pos(void);
void Draw_Big_Point(uint16_t x,uint16_t y);
void Touch_Adjust(void);


void get_calibrate_data(uint16_t* cal_data);
int  save_calibrate_to_flash(uint16_t* data);
unsigned int get_calibrate_state(void);


uint16_t TOUCH_DataFilter(uint8_t xy);
uint8_t TOUCH_ReadAdcXY(uint16_t *_usX, uint16_t *_usY);
void TOUCH_Scan(void);

#endif
