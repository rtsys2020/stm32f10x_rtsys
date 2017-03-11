/*
This file is part of the GSM3 communications library for Arduino
-- Multi-transport communications platform
-- Fully asynchronous
-- Includes code for the Arduino-Telefonica GSM/GPRS Shield V1
-- Voice calls
-- SMS
-- TCP/IP connections
-- HTTP basic clients

This library has been developed by Telef�nica Digital - PDI -
- Physical Internet Lab, as part as its collaboration with
Arduino and the Open Hardware Community. 

September-December 2012

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
https://github.com/BlueVia/Official-Arduino
*/
#include "GSM3SoftSerial.h"
#include "GSM3IO.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include "pins_arduino.h"
#include <HardwareSerial.h>
#include <Arduino.h>

#define __XON__ 0x11
#define __XOFF__ 0x13

#define _GSMSOFTSERIALFLAGS_ESCAPED_ 0x01
#define _GSMSOFTSERIALFLAGS_SENTXOFF_ 0x02

//#if defined(__STM32F1__)
  HardwareSerial *port;
//#endif

//
// Lookup table
//
#define __PARAGRAPHGUARD__ 50


GSM3SoftSerial* GSM3SoftSerial::_activeObject=0;

GSM3SoftSerial::GSM3SoftSerial():

	cb(this)
{
	//comStatus=0;
	//waitingAnswer=false;
  port = &Serial2;
}

int GSM3SoftSerial::begin(long speed)
{
  port->begin(speed);

}

void GSM3SoftSerial::close()
 {
  port->end();
 }

size_t GSM3SoftSerial::write(uint8_t c)
{


  // Characters to be escaped under XON/XOFF control with Quectel
  if(c==0x11)
  {
      port->write(0x77);
    return port->write(0xEE);
  }

  if(c==0x13)
  {
      port->write(0x77);
    return port->write(0xEC);
  }

  if(c==0x77)
  {
      port->write(0x77);
    return port->write(0x88);
  }

  return port->write(c);
}


/*inline*/
void GSM3SoftSerial::tunedDelay(uint16_t delay) {



}



void GSM3SoftSerial::recv()
{
  uint8_t d = 0;
  if(port->available()){
      d=port->read();
      cb.write(d);
  }




}

//bool GSM3SoftSerial::keepThisChar(uint8_t* c)
//{
//  return true;
//}

void GSM3SoftSerial::spaceAvailable()
{

}


// This is here to avoid problems with Arduino compiler
void GSM3SoftSerialMgr::manageMsg(byte from, byte to){};

//#define PCINT1_vect _VECTOR(2)
//#undef PCINT1_vect



