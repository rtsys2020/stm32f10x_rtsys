/************************************************************************************
 * 	
 * 	Name    : SST25VF.cpp                        
 * 	Author  : Noah Shibley                         
 * 	Date    : Aug 17th, 2013                                   
 * 	Version : 0.1                                              
 * 	Notes   : Based on SST code from: www.Beat707.com design. (Rugged Circuits and Wusik)      
 * 
 * 	
 * 
 ***********************************************************************************/

#include "SST25VF.h"

#define DEBUGPort Serial
SST25VF::SST25VF(SPIClass *spidev){
  _spidev = spidev;
}


void SST25VF::begin(int chipSelect,int writeProtect,int hold){

  //set pin #s
  FLASH_SSn = chipSelect;
  FLASH_Wp = writeProtect;
  FLASH_Hold = hold;

  _spidev->begin();
  _spidev->setDataMode(SPI_MODE0);

  pinMode(FLASH_Wp, OUTPUT);
  digitalWrite(FLASH_Wp, HIGH); //write protect off

  pinMode(FLASH_Hold, OUTPUT);
  digitalWrite(FLASH_Hold, HIGH); //mem hold off

  pinMode(FLASH_SSn, OUTPUT); //chip select
  digitalWrite(FLASH_SSn, HIGH);
  digitalWrite(FLASH_SSn, LOW);
  _spidev->setBitOrder(MSBFIRST);

  init();
  readID();

}

void SST25VF::begin(int chipSelect){

  //set pin #s
  FLASH_SSn = chipSelect;


  _spidev->begin();
  _spidev->setDataMode(SPI_MODE0);


  pinMode(FLASH_SSn, OUTPUT); //chip select
  digitalWrite(FLASH_SSn, HIGH);
  digitalWrite(FLASH_SSn, LOW);
  _spidev->setBitOrder(MSBFIRST);

  init();
  readID();

}

void SST25VF::update(){


}

// ======================================================================================= //

void SST25VF::waitUntilDone()
{
  uint8_t data = 0;
  while (1)
    {
      digitalWrite(FLASH_SSn,LOW);
      (void) _spidev->transfer(0x05);
      data = _spidev->transfer(0);
      digitalWrite(FLASH_SSn,HIGH);
      if (!bitRead(data,0)) break;
      nop();
    }
}

// ======================================================================================= //

void SST25VF::init()
{
  enable();
  digitalWrite(FLASH_SSn,LOW);
  _spidev->transfer(0x50); //enable write status register instruction
  digitalWrite(FLASH_SSn,HIGH);
  delay(50);
  digitalWrite(FLASH_SSn,LOW);
  _spidev->transfer(0x01); //write the status register instruction
  _spidev->transfer(0x00);//value to write to register - xx0000xx will remove all block protection
  digitalWrite(FLASH_SSn,HIGH);
  delay(50);
  disable();
}

// ======================================================================================= //

void SST25VF::readID()
{
  uint8_t id, mtype, dev;
  enable();
  digitalWrite(FLASH_SSn,LOW);
  (void) _spidev->transfer(0x9F); // Read ID command
  id = _spidev->transfer(0);
  mtype = _spidev->transfer(0);
  dev = _spidev->transfer(0);
  char buf[16] = {0};
  sprintf(buf, "%02X %02X %02X", id, mtype, dev);
  DEBUGPort.print("SPI ID ");
  DEBUGPort.println(buf);
  digitalWrite(FLASH_SSn,HIGH);
  disable();
}

// ======================================================================================= //

void SST25VF::totalErase()
{
  enable();
  digitalWrite(FLASH_SSn,LOW);
  _spidev->transfer(0x06);//write enable instruction
  digitalWrite(FLASH_SSn,HIGH);
  nop();
  digitalWrite(FLASH_SSn, LOW); 
  (void) _spidev->transfer(0x60); // Erase Chip //
  digitalWrite(FLASH_SSn, HIGH);
  waitUntilDone();
  disable();
}

// ======================================================================================= //

void SST25VF::setAddress(uint32_t addr)
{
  (void) _spidev->transfer(addr >> 16);
  (void) _spidev->transfer(addr >> 8);
  (void) _spidev->transfer(addr);
}

// ======================================================================================= //

void SST25VF::readInit(uint32_t address)
{
  enable();
  digitalWrite(FLASH_SSn,LOW);
  (void) _spidev->transfer(0x03); // Read Memory - 25/33 Mhz //
  setAddress(address);
}

// ======================================================================================= //

uint8_t SST25VF::readNext() { 
  return _spidev->transfer(0);

}

// ======================================================================================= //

void SST25VF::readFinish()
{
  digitalWrite(FLASH_SSn,HIGH);
  disable();
}

// ======================================================================================= //

void SST25VF::writeByte(uint32_t address, uint8_t data)
{
  enable();
  digitalWrite(FLASH_SSn,LOW);
  _spidev->transfer(0x06);//write enable instruction
  digitalWrite(FLASH_SSn,HIGH);
  nop();
  digitalWrite(FLASH_SSn,LOW);
  (void) _spidev->transfer(0x02); // Write Byte //
  setAddress(address);
  (void) _spidev->transfer(data);
  digitalWrite(FLASH_SSn,HIGH);
  waitUntilDone();
  disable();
}

uint32_t SST25VF::writeArray(uint32_t address,const uint8_t dataBuffer[],uint16_t dataLength)
{
  for(uint16_t i=0;i<dataLength;i++)
    {
      writeByte((uint32_t)address+i,dataBuffer[i]);

    }
  return address + dataLength;
}

void SST25VF::readArray(uint32_t address,uint8_t dataBuffer[],uint16_t dataLength)
{
  readInit((address));

  for (uint16_t i=0; i<dataLength; ++i)
    {
      dataBuffer[i] = readNext();
    }

  readFinish();

}

// ======================================================================================= //

void SST25VF::sectorErase(uint8_t sectorAddress)
{
  enable();
  digitalWrite(FLASH_SSn,LOW);
  _spidev->transfer(0x06);//write enable instruction
  digitalWrite(FLASH_SSn,HIGH);
  nop();
  digitalWrite(FLASH_SSn,LOW);
  (void) _spidev->transfer(0x20); // Erase 4KB Sector //
  setAddress(4096UL*long(sectorAddress));
  digitalWrite(FLASH_SSn,HIGH);
  waitUntilDone();
  disable();
}
