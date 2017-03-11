
/**
 * @file axp209.cpp
 *
 * @date Jan 10, 2017
 * @author jupiter
 * @description
 */
#include "axp209.h"

AXP209_Charger::AXP209_Charger (uint8_t scl, uint8_t sda, uint8_t delay)
{
  wire = new TwoWire(scl,sda,5); //Wire(scl, sda, delay);
}
AXP209_Charger::AXP209_Charger (TwoWire* i2cwire)
{
  wire = i2cwire;
}

void
AXP209_Charger::begin (void)
{
  wire->begin(AXP209_ADD);
}


uint8_t
AXP209_Charger::readPathSetting (void)
{
  uint8_t pathval;
  pathval = readRegister8(30);
  return pathval;
}



uint16_t AXP209_Charger::readBatteryVoltage (void)
{
  uint16_t batVal;
  batVal = readRegister8(0x78);

  batVal = (batVal<<8)|readRegister8(0x79);

  return batVal;
}

// Read byte from register
uint8_t AXP209_Charger::readRegister8(uint8_t reg)
{
  uint8_t value;
  wire->beginTransmission(AXP209_ADD);
  wire->write(reg);
  wire->endTransmission();

  wire->beginTransmission(AXP209_ADD);
  wire->requestFrom(AXP209_ADD, 1);

  uint32_t t1=systick_uptime();
  while(!wire->available()) {
      if(systick_uptime()>t1+300)
        {
          return 0;
        }
  };

  value = wire->read();
  wire->endTransmission();

  return value;
}


void AXP209_Charger::writeRegister8(uint8_t reg, uint8_t value)
{
  wire->beginTransmission(AXP209_ADD);

  wire->write(reg);
  wire->write(value);
  wire->endTransmission();
}

uint8_t AXP209_Charger::powerOutputControlRead(void){
  uint8_t pathval;
  pathval = readRegister8(0x57);
  return pathval;
}
void AXP209_Charger::powerOutputControlWrite(uint8_t val){
  uint8_t pathval;
//  pathval = readRegister8(0x00);
  writeRegister8(0x57,val&0x5F);
  return ;
}

uint8_t AXP209_Charger::powerInputStatus(void)
{
  uint8_t pathval;
  pathval = readRegister8(0x00);
  return pathval;
}
