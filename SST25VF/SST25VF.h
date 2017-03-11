/************************************************************************************
 * 	
 * 	Name    : SST25VF.h                        
 * 	Author  : Noah Shibley                         
 * 	Date    : Aug 17th, 2013                                    
 * 	Version : 0.1                                              
 * 	Notes   : Nor Serial Flash 16Mbit (2Mbytes) - S71295 (SST) 2097152 bytes.
 *			  Based on SST code from: www.Beat707.com design. (Rugged Circuits and Wusik)
 *			  Should work for SST25VF004, SST25VF016, SST25VF064, etc.                 
 * 
 * 	
 * 
 ***********************************************************************************/


//include guard
#ifndef SST25VF_H
#define SST25VF_H

#include <SPI.h>

#define FLASH_MAX_BYTES 2097152 //this chip contains this much storage
#define FLASH_MAX_SECTOR 512 //each sector is 4096 bytes

class SST25VF {
 
public: 

	SST25VF(SPIClass *spidev);
    void begin(int chipSelect,int writeProtect,int hold);
    void begin(int chipSelect);
    void update();
	void readID();
	void totalErase();
	void sectorErase(uint8_t sectorAddress);
	
	void readInit(uint32_t address);
	uint8_t readNext();
	void readFinish();
	void readArray(uint32_t address,uint8_t dataBuffer[],uint16_t dataLength);
	
	void writeByte(uint32_t address, uint8_t data);
	uint32_t writeArray(uint32_t address,const uint8_t dataBuffer[],uint16_t dataLength);
	
	

private: 
	SPIClass *_spidev;
	void init();
	char buf[16];
	uint8_t FLASH_SSn; //chip select pin
	uint8_t FLASH_Wp;  //Write protection pin
	uint8_t FLASH_Hold; //read/write hold pin (pause)

	
	void waitUntilDone();
	void setAddress(uint32_t addr);
	void enable()    { _spidev->setBitOrder(MSBFIRST); nop(); }
	void disable()   { _spidev->setBitOrder(LSBFIRST); nop(); }
	inline void volatile nop(void) { asm __volatile__ ("nop"); }
	
 
}; 
 
#endif
