/**
 * @file RCSwitchTask.cpp
 *
 * @date Dec 20, 2016
 * @author jupiter
 * @description
 */

#include <RCSwitch.h>
#include <HardwareSerial.h>

RCSwitch mySwitch = RCSwitch();

#define DebugPort  USBserial


static const char* bin2tristate(const char* bin);
static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength);
void output(unsigned long decimal, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol);
/**
 * @brief  Toggle LED2 thread
 * @param  argument not used
 * @retval None
 */
void RCSwitchTask(void const *argument)
{

  (void) argument;
  DebugPort.println("start rc-switch");
  mySwitch.enableReceive(PA7);  // Receiver on interrupt 0 => that is pin #2

  for (;;)
    {
      DebugPort.println("start loop");
      if (mySwitch.available()) {
          output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
          mySwitch.resetAvailable();
      }
      delay(1000);
    }
}


void output(unsigned long decimal, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol) {

  if (decimal == 0) {
      DebugPort.print("Unknown encoding.");
  } else {
      const char* b = dec2binWzerofill(decimal, length);
      DebugPort.print("Decimal: ");
      DebugPort.print(decimal);
      DebugPort.print(" (");
      DebugPort.print( length );
      DebugPort.print("Bit) Binary: ");
      DebugPort.print( b );
      DebugPort.print(" Tri-State: ");
      DebugPort.print( bin2tristate( b) );
      DebugPort.print(" PulseLength: ");
      DebugPort.print(delay);
      DebugPort.print(" microseconds");
      DebugPort.print(" Protocol: ");
      DebugPort.println(protocol);
  }

  DebugPort.print("Raw data: ");
  for (unsigned int i=0; i<= length*2; i++) {
      DebugPort.print(raw[i]);
      DebugPort.print(",");
  }
  DebugPort.println();
  DebugPort.println();
}

static const char* bin2tristate(const char* bin) {
  static char returnValue[50];
  int pos = 0;
  int pos2 = 0;
  while (bin[pos]!='\0' && bin[pos+1]!='\0') {
      if (bin[pos]=='0' && bin[pos+1]=='0') {
          returnValue[pos2] = '0';
      } else if (bin[pos]=='1' && bin[pos+1]=='1') {
          returnValue[pos2] = '1';
      } else if (bin[pos]=='0' && bin[pos+1]=='1') {
          returnValue[pos2] = 'F';
      } else {
          return "not applicable";
      }
      pos = pos+2;
      pos2++;
  }
  returnValue[pos2] = '\0';
  return returnValue;
}

static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
  static char bin[64];
  unsigned int i=0;

  while (Dec > 0) {
      bin[32+i++] = ((Dec & 1) > 0) ? '1' : '0';
      Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
      if (j >= bitLength - i) {
          bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
      } else {
          bin[j] = '0';
      }
  }
  bin[bitLength] = '\0';

  return bin;
}



