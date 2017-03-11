/**
 * @file DummyTask.cpp
 *
 * @date Dec 31, 2016
 * @author jupiter
 * @description
 */

#include <Arduino.h>
#include "SerialFlash.h"
#include "axp209.h"
//#include "ArduinoJson.h"
#include "cJSON.h"
#include "personInfo.h"

#define RIGHT_KEY PA3
#define CANCEL_KEY  PA8
#define DOWN_KEY  PB0
#define LEFT_KEY  PB1
#define OK_KEY  PB2
#define UP_KEY  PB8
#define MENU_KEY  PA1
#define LED1_PIN  PB3
#define LED2_PIN  PB5


uint8_t scrollKey=0;
uint8_t okKey=0;
uint8_t cancelKey=0;

uint8_t pinDown=0;


void clearKeys(void)
{
  okKey=0;
  cancelKey=0;
}

void KeypadTask(void const *argument)
{
  pinMode(LED1_PIN,OUTPUT);//LED1
  pinMode(LED2_PIN,OUTPUT);//LED2

  pinMode(DOWN_KEY, INPUT_FLOATING);//down
  pinMode(LEFT_KEY, INPUT_FLOATING);//left
  pinMode(OK_KEY, INPUT_FLOATING);//ok
  pinMode(UP_KEY, INPUT_FLOATING);//up
  pinMode(PB9, INPUT_FLOATING);
  //
  //    //  pinMode(PA0, INPUT_PULLUP);
  pinMode(MENU_KEY, INPUT_FLOATING);//menu
  pinMode(RIGHT_KEY, INPUT_FLOATING);//right
  pinMode(CANCEL_KEY, INPUT_FLOATING);//cancel

  //
  while(1){
      if( digitalRead(PB0)==0 )
        scrollKey +=1;
      if( digitalRead(PB9)==0 )
        okKey =1;

      if( digitalRead(PB8)==0 )
        cancelKey =1;
      digitalWrite(LED1_PIN,1);
      digitalWrite(LED2_PIN,1);

      //        digitalWrite(PB6,1);
      //        digitalWrite(PB7,1);

      delay (500);
      USBserial.print("*");

      //        digitalWrite(PB6,0);
      //        digitalWrite(PB7,0);


      digitalWrite(LED1_PIN,0);
      digitalWrite(LED2_PIN,0);

      delay(500);
  }
}






