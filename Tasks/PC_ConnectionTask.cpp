/**
 * @file PC_ConnectionTask.cpp
 *
 * @date Mar 10, 2017
 * @author jupiter
 * @description
 */


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


const char respOK[]="{\"cmd\":\"respOK\",\"data\":{\"value\":\"success\"}}";
const char respErr1[]="{\"cmd\":\"respERR\",\"data\":{\"value\":\"Data Error\"}}";
const char respErr2[]="{\"cmd\":\"respERR\",\"data\":{\"value\":\"Parse Error\"}}";
const char deviseStatus[]="{\"cmd\":\"deviceStaus\",\"data\":{\"value\":\"Ready\"}}";
const char flashStatus[]="{\"cmd\":\"flashStaus\",\"data\":{\"value\":\"Ready\"}}";



void addUser(cJSON* root);


//#include "SerialFlash.h"

void PC_ConnectionTask(void const *argument)
{
  pinMode(PB3,OUTPUT);//LED1
  pinMode(PB5,OUTPUT);//LED2





  while(USBserial.available()==0){
      digitalWrite(PB3,0);
      digitalWrite(PB5,0);

      delay(1000);
      digitalWrite(PB3,1);
      digitalWrite(PB5,1);
      delay(1000);
  }


  USBserial.println(deviseStatus);
  SPIClass1.begin();
  SPIClass1.setClockDivider(SPI_CLOCK_DIV2); //4MHz
  SPIClass1.setDataMode(SPI_MODE0);



  USBserial.flush();
  digitalWrite(PB3,0);
  digitalWrite(PB5,0);
  //  SerialFlash.begin(PA4);
  //  SerialFlash.eraseAll();

  //Flash LED at 1Hz while formatting
  while (!SerialFlash.ready()) {
      delay(200);
      digitalWrite(PB5, HIGH);
      delay(200);
      digitalWrite(PB5, LOW);
  }
  uint8_t tmp[10];
  if(0){
      SerialFlash.readID(tmp);
      USBserial.println("flash ready");
      int chipsize = SerialFlash.capacity(tmp);
      USBserial.print(chipsize);
      //sector size 4K
      //block size 32K 64K
      USBserial.print("  Block Size:   ");
      int blocksize = SerialFlash.blockSize();
      USBserial.print(blocksize);
      USBserial.println(" bytes");

      SerialFlash.wakeup();

      SerialFlash.eraseSector(0);

      USBserial.println(flashStatus);

      //      USBserial.print(" erased sector ");
      //      SerialFlash.write(0,"salam",5);
      //      uint8_t buff[10];
      //      SerialFlash.read(0,buff,5);
      //      USBserial.println(buff[0]);
  }



  uint8_t pageindex=0;
  while(1){
      digitalWrite(PB3,1);
      digitalWrite(PB5,1);

      if(USBserial.available()>0){
          String st = USBserial.readStringUntil('\n');
          cJSON *root = NULL;
          //char content[]="{\"cmd\":\"getstatus\",\"data\":{\"name\":\"saeed\",\"family\":\"reza\",\"finger1\":12.0,\"finger2\":13.0}}";
          //{"cmd":"setUser","data":{"name":"saeed","family":"rezaeefa","finger1":12.0,"finger2":13.0}}
          //{"cmd":"setUser","data":{"name":"saeed","finger1":12.0,"finger2":13.0}}
          root = cJSON_Parse(st.c_str());
          if (root == NULL)
            {
              USBserial.println(respErr1);
            }
          else{
              cJSON *cmd = NULL;
              cmd = cJSON_GetObjectItem(root, "cmd");
              if(cmd ==NULL)
                {
                  USBserial.println(respErr1);
                  continue;
                }
              else{
                  //USBserial.println(respOK);
                  if(String("setUser")==String(cmd->valuestring))
                    {
                      addUser(root);
                      free(root);
                      USBserial.println(respOK);
                    }
                  else if(String("saveUser")==String(cmd->valuestring))
                    {
//                      USBserial.println("save setting");
                      SerialFlash.writePage(0,"55AA557e",9);
                      USBserial.println(respOK);
                    }
              }
          }
          USBserial.flush();
      }
      delay (500);
      digitalWrite(PB3,0);
      digitalWrite(PB5,0);

      delay (500);

  }
}


void addUser(cJSON* root){
  uint8_t success=0;
  static uint8_t pageindex=0;
  cJSON* data = cJSON_CreateArray();
  data = cJSON_GetObjectItem(root, "data");
  if(data !=NULL){
      cJSON *name = NULL;
      name = cJSON_GetObjectItem(data, "name");
      if(name !=NULL){
          success++;
          USBserial.println(name->valuestring);
      }
      else
        free(name);
      cJSON *family = NULL;

      family = cJSON_GetObjectItem(data, "family");
      if(family !=NULL){
          success++;
          USBserial.println(family->valuestring);
      }
      else
        free(family);

      cJSON *finger1 = NULL;
      finger1 = cJSON_GetObjectItem(data, "finger1");
      if(finger1 !=NULL){
          success++;
          USBserial.println(finger1->valuedouble);
      }
      else
        free(finger1);

      cJSON *finger2 = NULL;
      finger2 = cJSON_GetObjectItem(data, "finger2");
      if(finger2 !=NULL){
          success++;
          USBserial.println(finger2->valuedouble);
      }
      else
        free(finger2);

      free(data);

      if(success==4){
          USBserial.println(respOK);
          //save data
          personInfo_t person;
          memcpy(person.name,name,strlen(name->valuestring));
          memcpy(person.family,family,strlen(family->valuestring));
          person.finger1 = finger1->valuedouble;
          person.finger2 = finger2->valuedouble;
          //write to page n
          //                      SerialFlash.writePage(pageindex,(uint8_t *)&person,sizeof(person));
          pageindex++;
      }
      else{
          USBserial.println(respErr2);
      }
  }
}
