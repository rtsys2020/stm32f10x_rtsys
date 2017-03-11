/**
 * @file GUITask.cpp
 *
 * @date Dec 20, 2016
 * @author jupiter
 * @description
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//Adafruit_ST7735 tft = Adafruit_ST7735(PB12, PA10, PA8);
Adafruit_ST7735 tft = Adafruit_ST7735(PB12, PA2, PA0);

extern uint8_t scrollKey,okKey,cancelKey;
#include "personInfo.h"


personInfo_t users[]={\
    {"ali","rezaee",10,8}\
    ,{"saeed","rahimi",8,5}\
    ,{"javad","alimoradi",12,8}\
    ,{"javad2","alimoradi1",12,8}\
    ,{"javad3","alimoradi2",12,8}\
    ,{"javad4","alimoradi3",12,8}\
    ,{0,0,0,0}\
};

/* GIMP RGBA C-Source image dump (icon.c) */





float degre=22;
float distance=12;




void clearKeys(void);
uint16_t backGroungColor=ST7735_BLACK;


void GuiTask(void const *argument)
{


  uint8_t cmd1[]={0x35,0x00,0x04};
  uint8_t buff[10];
  uint32_t count = 0;
  tft.initR(INITR_BLACKTAB);
  tft.invertDisplay(0);
  tft.setRotation(1);

  tft.fillScreen(backGroungColor);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_CYAN);

//  tft.drawXBitmap(80,60,gimp_image.pixel_data,100,100);

  uint8_t page=0;
  uint8_t row=3;
  uint8_t firstDraw=1;
  for (;;)
    {


      if(firstDraw==1){
          tft.setTextSize(2);
          tft.fillScreen(backGroungColor);

          tft.setCursor(5,5);
          tft.print(users[page*row+0].name);
          tft.print(" ");
          tft.print(users[page*row+0].family);
          tft.print(digitalRead(PA0));


          tft.setCursor(5,42);
          tft.print(users[page*row+1].name);
          tft.print(" ");
          tft.print(users[page*row+1].family);

          tft.setCursor(5,82);
          tft.print(users[page*row+2].name);
          tft.print(" ");
          tft.print(users[page*row+2].family);

          firstDraw=0;
      }

      if(scrollKey%4==0){

          tft.drawRoundRect(1,1,158,38,5,ST7735_BLUE);
          tft.drawRoundRect(1,41,158,38,5,ST7735_CYAN);
          tft.drawRoundRect(1,81,158,38,5,ST7735_CYAN);
      }
      else if(scrollKey%4==1){

          tft.drawRoundRect(1,1,158,38,5,ST7735_CYAN);
          tft.drawRoundRect(1,41,158,38,5,ST7735_BLUE);
          tft.drawRoundRect(1,81,158,38,5,ST7735_CYAN);
      }
      else if(scrollKey%4==2){

          tft.drawRoundRect(1,1,158,38,5,ST7735_CYAN);
          tft.drawRoundRect(1,41,158,38,5,ST7735_CYAN);
          tft.drawRoundRect(1,81,158,38,5,ST7735_BLUE);
      }
      else{
          firstDraw=1;
          page+=1;
          page=page%2;
      }
      if(okKey==1)
        {
          uint8_t index=scrollKey%3;

          tft.setTextSize(1);
          tft.fillScreen(backGroungColor);
          okKey=0;
          tft.setCursor(5,5);
          tft.print("Name:  \t");
          tft.println(users[index].name);

          tft.setCursor(5,35);
          tft.print("Family:  \t");
          tft.println(users[index].family);

          tft.setCursor(5,75);
          tft.print("Finger1:  \t");
          tft.println(users[index].finger1);

          tft.setCursor(5,110);
          tft.print("Finger2:  \t");
          tft.println(users[index].finger2);
//          delay(300);
//          clearKeys();
          while(cancelKey==0)
            {
              if(okKey==1)
                {
                  tft.setTextSize(1);
                  tft.fillScreen(backGroungColor);
                  okKey=0;
                  tft.setCursor(5,5);
                  tft.print("degre:");
                  tft.println(degre);

                  tft.setCursor(5,35);
                  tft.print("Distance:");
                  tft.println(distance);
                }
              delay(100);
            }
          clearKeys();

          firstDraw=1;

        }
      //      tft.println(count++,10);
      delay(1000);
    }
}

