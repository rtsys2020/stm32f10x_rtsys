/**
 * @file axp209.h
 *
 * @date Jan 10, 2017
 * @author jupiter
 * @description
 */

#ifndef AXP209_H_
#define AXP209_H_
#include <stdint.h>
#include "Wire.h"

#define AXP209_READ_ADD 0x69

#define AXP209_WRITE_ADD 0x68
#define AXP209_ADD 0x34

typedef enum{
  ACIN_OR_VBUS_SOURCE_BIT,
  ACIN_AND_VBUS_SHORTED_BIT,
  BATT_DIRECTION_BIT,
  VBUS_DIRECTION_BIT,
  VBUS_AVAILABLE_BIT,
  VBUS_PRESENCE_BIT,
  ACIN_AVAILABLE_BIT,
  ACIN_PRESENCE_BIT
}power_input_status_bit;

typedef enum{
  EXTEN_SWITCH_BIT,
  DC_DC3_SWITCH_BIT,
  LDO2_SWITCH_BIT,
  LDO4_SWITCH_BIT,
  DC_DC2_SWITCH_BIT,
  RESERVED1,
  LDO3_SWITCH_BIT,
  RESERVED2
}power_output_control_bit;

class AXP209_Charger{

public:
 AXP209_Charger(uint8_t scl, uint8_t sda, uint8_t delay=5);
 AXP209_Charger(TwoWire *i2cwire);

 void begin(void);
//VBUS-IPSOUT Path setting register reg  30  default 0x60
 uint8_t readPathSetting(void);
 uint16_t readBatteryVoltage(void);
 uint8_t powerInputStatus(void);
 uint8_t powerOutputControlRead(void);
 void powerOutputControlWrite(uint8_t);
private:
 private:
 TwoWire *wire;
 void writeRegister8(uint8_t reg, uint8_t value);
 uint8_t readRegister8(uint8_t reg);
};


//
//ADC Measurement Values
//First register is high 8 bits, second is lower 4 (lower 5 for battery discharge current).
//Channel Registers                             000h  Step  FFFh
//ACIN Voltage                  56h, 57h  0 mV  1.7 mV  6.9615 V
//ACIN Current                  58h, 59h  0 mA  0.625 mA  2.5594 A
//VBUS voltage                  5Ah, 5Bh  0 mV  1.7 mV  6.9615 V
//VBUS Current                  5Ch, 5Dh  0 mA  0.375 mA  1.5356 A
//Internal Temperature          5Eh, 5Fh  -144.7 C  0.1 C 264.8 C
//Temperature Sensor Voltage    62h, 63h  0 mV  0.8 mV  3.276 V
//APS (IPSOUT) Voltage          7Eh, 7Fh  0 mV  1.4 mV  5.733 V
//Battery Voltage               78h, 79h  0 mV  1.1 mV  4.5045 V
//Battery Discharge Current     7Ah, 7Bh  0 mA  0.5 mA  4.095 A
//Battery Charge Current        7Ch, 7Dh  0 mA  0.5 mA  4.095 A
//
//


/**
 *
 *
http://linux-sunxi.org/AXP209#GPIO



First, install bc arbitrary precision numeric processing language :

sudo apt-get install bc

Now there are two scripts for monitoring AXP209 :

battery_info.sh
#!/bin/sh
# This program gets the battery info from PMU
# Voltage and current charging/discharging
#
# Nota : temperature can be more than real because of self heating
#######################################################################
# Copyright (c) 2014 by RzBo, Bellesserre, France
#
# Permission is granted to use the source code within this
# file in whole or in part for any use, personal or commercial,
# without restriction or limitation.
#
# No warranties, either explicit or implied, are made as to the
# suitability of this code for any purpose. Use at your own risk.
#######################################################################

# force ADC enable for battery voltage and current
i2cset -y -f 0 0x34 0x82 0xC3

################################
#read Power status register @00h
POWER_STATUS=$(i2cget -y -f 0 0x34 0x00)
#echo $POWER_STATUS

BAT_STATUS=$(($(($POWER_STATUS&0x02))/2))  # divide by 2 is like shifting rigth 1 times
#echo $(($POWER_STATUS&0x02))
echo « BAT_STATUS= »$BAT_STATUS
# echo $BAT_STATUS

################################
#read Power OPERATING MODE register @01h
POWER_OP_MODE=$(i2cget -y -f 0 0x34 0x01)
#echo $POWER_OP_MODE

CHARG_IND=$(($(($POWER_OP_MODE&0x40))/64))  # divide by 64 is like shifting rigth 6 times
#echo $(($POWER_OP_MODE&0x40))
echo « CHARG_IND= »$CHARG_IND
# echo $CHARG_IND

BAT_EXIST=$(($(($POWER_OP_MODE&0x20))/32))  # divide by 32 is like shifting rigth 5 times
#echo $(($POWER_OP_MODE&0x20))
echo « BAT_EXIST= »$BAT_EXIST
# echo $BAT_EXIST

################################
#read Charge control register @33h
CHARGE_CTL=$(i2cget -y -f 0 0x34 0x33)
echo « CHARGE_CTL= »$CHARGE_CTL
# echo $CHARGE_CTL

################################
#read Charge control register @34h
CHARGE_CTL2=$(i2cget -y -f 0 0x34 0x34)
echo « CHARGE_CTL2= »$CHARGE_CTL2
# echo $CHARGE_CTL2

################################
#read battery voltage    79h, 78h    0 mV -> 000h,    1.1 mV/bit    FFFh -> 4.5045 V
BAT_VOLT_LSB=$(i2cget -y -f 0 0x34 0x79)
BAT_VOLT_MSB=$(i2cget -y -f 0 0x34 0x78)

#echo $BAT_VOLT_MSB $BAT_VOLT_LSB

BAT_BIN=$(( $(($BAT_VOLT_MSB << 4)) | $(($(($BAT_VOLT_LSB & 0xF0)) >> 4)) ))

BAT_VOLT=$(echo « ($BAT_BIN*1.1) »|bc)
echo « Battery voltage = « $BAT_VOLT »mV »

###################
#read Battery Discharge Current    7Ah, 7Bh    0 mV -> 000h,    0.5 mA/bit    FFFh -> 4.095 V
BAT_IDISCHG_LSB=$(i2cget -y -f 0 0x34 0x7B)
BAT_IDISCHG_MSB=$(i2cget -y -f 0 0x34 0x7A)

#echo $BAT_IDISCHG_MSB $BAT_IDISCHG_LSB

BAT_IDISCHG_BIN=$(( $(($BAT_IDISCHG_MSB << 4)) | $(($(($BAT_IDISCHG_LSB & 0xF0)) >> 4)) ))

BAT_IDISCHG=$(echo « ($BAT_IDISCHG_BIN*0.5) »|bc)
echo « Battery discharge current = « $BAT_IDISCHG »mA »

###################
#read Battery Charge Current    7Ch, 7Dh    0 mV -> 000h,    0.5 mA/bit    FFFh -> 4.095 V
BAT_ICHG_LSB=$(i2cget -y -f 0 0x34 0x7D)
BAT_ICHG_MSB=$(i2cget -y -f 0 0x34 0x7C)

#echo $BAT_ICHG_MSB $BAT_ICHG_LSB

BAT_ICHG_BIN=$(( $(($BAT_ICHG_MSB << 4)) | $(($(($BAT_ICHG_LSB & 0xF0)) >> 4)) ))

BAT_ICHG=$(echo « ($BAT_ICHG_BIN*0.5) »|bc)
echo « Battery charge current = « $BAT_ICHG »mA »

power_status.sh
#!/bin/sh
# This program gets the power status (AC IN or BAT)
# I2C interface with AXP209
#
#######################################################################
# Copyright (c) 2014 by RzBo, Bellesserre, France
#
# Permission is granted to use the source code within this
# file in whole or in part for any use, personal or commercial,
# without restriction or limitation.
#
# No warranties, either explicit or implied, are made as to the
# suitability of this code for any purpose. Use at your own risk.
#######################################################################

#read Power status register @00h
POWER_STATUS=$(i2cget -y -f 0 0x34 0x00)
#echo $POWER_STATUS

# bit 7 : Indicates ACIN presence 0: ACIN does not exist; 1: ACIN present
#echo « bit 7 : Indicates ACIN presence 0: ACIN does not exist; 1: ACIN present »
#echo « bit 2 : Indicates that the battery current direction 0: battery discharge; 1: The battery is charged »

AC_STATUS=$(($(($POWER_STATUS&0x80))/128))  # divide by 128 is like shifting rigth 8 times
#echo $(($POWER_STATUS&0x80))
echo « AC_STATUS= »$AC_STATUS
# echo $AC_STATUS

Examples :

On AC, not charging :

AC_STATUS=1

BAT_STATUS=0
CHARG_IND=0
BAT_EXIST=1
CHARGE_CTL=0xd0
CHARGE_CTL2=0x47
Battery voltage = 4176.7mV
Battery discharge current = 0mA
Battery charge current = 0mA

On AC, charging :

AC_STATUS=1

BAT_STATUS=0
CHARG_IND=1
BAT_EXIST=1
CHARGE_CTL=0xd0
CHARGE_CTL2=0x47
Battery voltage = 4188.8mV
Battery discharge current = 200.0mA
Battery charge current = 0mA

On battery :

AC_STATUS=0

BAT_STATUS=0
CHARG_IND=0
BAT_EXIST=1
CHARGE_CTL=0xd0
CHARGE_CTL2=0x47
Battery voltage = 4030.4mV
Battery discharge current = 0mA
Battery charge current = 160.0mA



 */

#endif /* AXP209_H_ */
