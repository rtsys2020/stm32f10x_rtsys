/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SerialFlash.h"
#include "util/SerialFlash_directwrite.h"




uint16_t SerialFlashChip::dirindex = 0;
uint8_t SerialFlashChip::flags = 0;
uint8_t SerialFlashChip::busy = 0;

static volatile IO_REG_TYPE *cspin_basereg;
static IO_REG_TYPE cspin_bitmask;



/**
 *
 */
void SerialFlashChip::wait(void)
{
  uint32_t status;
  //Serial.print("wait-");
  while (1) {
      // SRP0 SEC TB  BP2 BP1 BP0 WEL BUSY
      // SUS  CMP LB3 LB2 LB1 R   QE  SRP1
      SPI.beginTransaction(SPICONFIG);
      CSASSERT();
      if (flags & FLAG_STATUS_CMD70) {
          // some Micron chips require this different
          // command to detect program and erase completion
          SPI.transfer(0x70);
          status = SPI.transfer(0);
          CSRELEASE();
          SPI.endTransaction();
          //Serial.printf("b=%02x.", status & 0xFF);
          if ((status & 0x80)) break;
      } else {
          // all others work by simply reading the status reg
          SPI.transfer(0x05);
          status = SPI.transfer(0xff);
          CSRELEASE();
          SPI.endTransaction();
          //Serial.printf("b=%02x.", status & 0xFF);
          if (!(status & 1)) break;
      }
  }
  busy = 0;
  //Serial.println();
}
/**
 *
 * @param reg
 * @return
 */
uint32_t SerialFlashChip::readReg8(uint8_t reg)
{
  uint32_t status;
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  // all others work by simply reading the status reg
  SPI.transfer(reg);
  status = SPI.transfer(0xff);
  CSRELEASE();
  SPI.endTransaction();
  busy = 0;
  return status;
}
/**
 *
 * @param reg
 * @return
 */
uint32_t SerialFlashChip::readReg16(uint8_t reg)
{
  uint32_t status;
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  // all others work by simply reading the status reg
  SPI.transfer(reg);
  status = SPI.transfer(0xff)<<8;
  status |= SPI.transfer(0xff);
  CSRELEASE();
  SPI.endTransaction();
  busy = 0;
  return status;
}
/**
 *
 */
void SerialFlashChip::writeEn(void)
{
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  // write enable command
  SPI.transfer(0x06);
  SPI.endTransaction();
}
/**
 *
 * @param reg
 * @return
 */
uint32_t SerialFlashChip::readReg32(uint8_t reg)
{
  uint32_t status;
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  // all others work by simply reading the status reg
  SPI.transfer(reg);
  status = SPI.transfer(0xff)<<24;
  status |= SPI.transfer(0xff)<<16;
  status |= SPI.transfer(0xff)<<8;
  status |= SPI.transfer(0xff);
  CSRELEASE();
  SPI.endTransaction();
  busy = 0;
  return status;
}
/**
 *
 * @param addr
 * @param buf
 * @param len
 * @return
 */
bool SerialFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{
  uint8_t *p = (uint8_t *)buf;
  uint8_t b, f, status, cmd;

  memset(p, 0, len);
  f = flags;
  SPI.beginTransaction(SPICONFIG);
  b = busy;
  if (b) {
      // read status register ... chip may no longer be busy
      CSASSERT();
      if (flags & FLAG_STATUS_CMD70) {
          SPI.transfer(0x70);
          status = SPI.transfer(0);
          if ((status & 0x80)) b = 0;
      } else {
          SPI.transfer(0x05);
          status = SPI.transfer(0);
          if (!(status & 1)) b = 0;
      }
      CSRELEASE();
      if (b == 0) {
          // chip is no longer busy :-)
          busy = 0;
      } else if (b < 3) {
          // TODO: this may not work on Spansion chips
          // which apparently have 2 different suspend
          // commands, for program vs erase
          CSASSERT();
          SPI.transfer(0x06); // write enable (Micron req'd)
          CSRELEASE();
          delayMicroseconds(1);
          cmd = 0x75; //Suspend program/erase for almost all chips
          // but Spansion just has to be different for program suspend!
          if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x85;
          CSASSERT();
          SPI.transfer(cmd); // Suspend command
          CSRELEASE();
          if (f & FLAG_STATUS_CMD70) {
              // Micron chips don't actually suspend until flags read
              CSASSERT();
              SPI.transfer(0x70);
              do {
                  status = SPI.transfer(0);
              } while (!(status & 0x80));
              CSRELEASE();
          } else {
              CSASSERT();
              SPI.transfer(0x05);
              do {
                  status = SPI.transfer(0);
              } while ((status & 0x01));
              CSRELEASE();
          }
      } else {
          // chip is busy with an operation that can not suspend
          SPI.endTransaction();	// is this a good idea?
          wait();			// should we wait without ending
          b = 0;			// the transaction??
          SPI.beginTransaction(SPICONFIG);
      }
  }
  do {
      uint32_t rdlen = len;
      if (f & FLAG_MULTI_DIE) {
          if ((addr & 0xFE000000) != ((addr + len - 1) & 0xFE000000)) {
              rdlen = 0x2000000 - (addr & 0x1FFFFFF);
          }
      }
      CSASSERT();
      // TODO: FIFO optimize....
      if (f & FLAG_32BIT_ADDR) {
          SPI.transfer(0x03);
          SPI.transfer16(addr >> 16);
          SPI.transfer16(addr);
      } else {
          SPI.transfer16(0x0300 | ((addr >> 16) & 255));
          SPI.transfer16(addr);
      }
      SPI.dmaTransfer(p,p, rdlen);
      CSRELEASE();
      p += rdlen;
      addr += rdlen;
      len -= rdlen;
  } while (len > 0);
  if (b) {
      CSASSERT();
      SPI.transfer(0x06); // write enable (Micron req'd)
      CSRELEASE();
      delayMicroseconds(1);
      cmd = 0x7A;
      if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x8A;
      CSASSERT();
      SPI.transfer(cmd); // Resume program/erase
      CSRELEASE();
  }
  SPI.endTransaction();
  return true;
}
/**
 *
 * @param addr
 * @param buf
 * @param len
 * @return
 */
bool SerialFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  uint32_t max, pagelen;

  //Serial.printf("WR: addr %08X, len %d\n", addr, len);
  do {
      if (busy) wait();
      SPI.beginTransaction(SPICONFIG);
      CSASSERT();
      // write enable command
      SPI.transfer(0x06);
      CSRELEASE();
      max = 256 - (addr & 0xFF);
      pagelen = (len <= max) ? len : max;
      //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);
      CSASSERT();
      if (flags & FLAG_32BIT_ADDR) {
          SPI.transfer(0x02); // program page command
          SPI.transfer16(addr >> 16);
          SPI.transfer16(addr);
      } else {
          SPI.transfer16(0x0200 | ((addr >> 16) & 255));
          SPI.transfer16(addr);
      }
      addr += pagelen;
      len -= pagelen;
      do {
          SPI.transfer(*p++);
      } while (--pagelen > 0);
      CSRELEASE();
      busy = 1;
      SPI.endTransaction();
  } while (len > 0);
  return true;
}
/**
 *
 * @param addr
 * @param buf
 * @return
 */
bool SerialFlashChip::writePage(uint32_t addrpage, const void *buf,uint8_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  uint32_t max, pagelen,addr;
  addr = 0x100*addrpage;
  //Serial.printf("WR: addr %08X, len %d\n", addr, len);

  if (busy) wait();
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  // write enable command
  SPI.transfer(0x06);
  CSRELEASE();
  //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);
  CSASSERT();
  if (flags & FLAG_32BIT_ADDR) {
      SPI.transfer(0x02); // program page command
      SPI.transfer16(addr >> 16);
      SPI.transfer16(addr);
  } else {
      SPI.transfer16(0x0200 | ((addr >> 16) & 255));
      SPI.transfer16(addr);
  }
  pagelen=len;
  do {
      SPI.transfer(*p++);
  } while (--pagelen > 0);
  CSRELEASE();
  busy = 1;
  SPI.endTransaction();

  return true;
}

/**
 *
 */
void SerialFlashChip::eraseAll()
{
  if (busy) wait();
  uint8_t id[3];
  readID(id);
  //Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
  if (id[0] == 0x20 && id[2] >= 0x20 && id[2] <= 0x22) {
      // Micron's multi-die chips require special die erase commands
      //  N25Q512A	20 BA 20  2 dies  32 Mbyte/die   65 nm transitors
      //  N25Q00AA	20 BA 21  4 dies  32 Mbyte/die   65 nm transitors
      //  MT25QL02GC	20 BA 22  2 dies  128 Mbyte/die  45 nm transitors
      uint8_t die_count = 2;
      if (id[2] == 0x21) die_count = 4;
      uint8_t die_index = flags >> 6;
      //Serial.printf("Micron die erase %d\n", die_index);
      flags &= 0x3F;
      if (die_index >= die_count) return; // all dies erased :-)
      uint8_t die_size = 2;  // in 16 Mbyte units
      if (id[2] == 0x22) die_size = 8;
      SPI.beginTransaction(SPICONFIG);
      CSASSERT();
      SPI.transfer(0x06); // write enable command
      CSRELEASE();
      delayMicroseconds(1);
      CSASSERT();
      // die erase command
      SPI.transfer(0xC4);
      SPI.transfer16((die_index * die_size) << 8);
      SPI.transfer16(0x0000);
      CSRELEASE();
      //Serial.printf("Micron erase begin\n");
      flags |= (die_index + 1) << 6;
  } else {
      // All other chips support the bulk erase command
      SPI.beginTransaction(SPICONFIG);
      CSASSERT();
      // write enable command
      SPI.transfer(0x06);
      CSRELEASE();
      delayMicroseconds(1);
      CSASSERT();
      // bulk erase command
      SPI.transfer(0xC7);
      CSRELEASE();
      SPI.endTransaction();
  }
  busy = 3;
}
/**
 *
 * @param addr
 */
void SerialFlashChip::eraseBlock(uint32_t addr)
{
  uint8_t f = flags;
  if (busy) wait();
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  SPI.transfer(0x06); // write enable command
  CSRELEASE();
  delayMicroseconds(1);
  CSASSERT();
  if (f & FLAG_32BIT_ADDR) {
      SPI.transfer(0xD8);
      SPI.transfer16(addr >> 16);
      SPI.transfer16(addr);
  } else {
      SPI.transfer16(0xD800 | ((addr >> 16) & 255));
      SPI.transfer16(addr);
  }
  CSRELEASE();
  SPI.endTransaction();
  busy = 2;
}
/**
 *
 * @param addr
 * @return
 */
bool SerialFlashChip::eraseSector(uint32_t addr)
{
  uint8_t f = flags;
  if (busy) wait();
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  SPI.transfer(0x06); // write enable command
  CSRELEASE();
  delayMicroseconds(1);
  CSASSERT();
  if (f & FLAG_32BIT_ADDR) {
      SPI.transfer(0x20);
      SPI.transfer16(addr >> 16);
      SPI.transfer16(addr);
  } else {
      SPI.transfer16(0x2000 | ((addr >> 16) & 255));
      SPI.transfer16(addr);
  }
  CSRELEASE();
  SPI.endTransaction();
  busy = 3;
  return true;
}

/**
 *
 * @return
 */
bool SerialFlashChip::ready()
{
  uint32_t status;
  if (!busy) return true;
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  if (flags & FLAG_STATUS_CMD70) {
      // some Micron chips require this different
      // command to detect program and erase completion
      SPI.transfer(0x70);
      status = SPI.transfer(0);
      CSRELEASE();
      SPI.endTransaction();
      //Serial.printf("ready=%02x\n", status & 0xFF);
      if ((status & 0x80) == 0) return false;
  } else {
      // all others work by simply reading the status reg
      SPI.transfer(0x05);
      status = SPI.transfer(0);
      CSRELEASE();
      SPI.endTransaction();
      //Serial.printf("ready=%02x\n", status & 0xFF);
      if ((status & 1)) return false;
  }
  busy = 0;
  if (flags & 0xC0) {
      // continue a multi-die erase
      eraseAll();
      return false;
  }
  return true;
}

/**
 *
 * @param pin
 * @return
 */
bool SerialFlashChip::begin(uint8_t pin)
{
  unsigned char id[10];
  uint8_t f;
  uint32_t size;

  cspin_basereg = PIN_TO_BASEREG(pin);
  cspin_bitmask = PIN_TO_BITMASK(pin);
  SPI.begin();
  pinMode(pin, OUTPUT);
  CSRELEASE();
  USBserial.println("flash init:");
  readID(id);

  USBserial.print(id[0],HEX);
  USBserial.print(":");
  USBserial.print(id[1],HEX);
  USBserial.print(":");
  USBserial.println(id[2],HEX);



  f = 0;
  size = capacity(id);
  if (size > 16777216) {
      // more than 16 Mbyte requires 32 bit addresses
      f |= FLAG_32BIT_ADDR;
      SPI.beginTransaction(SPICONFIG);
      if (id[0] == ID0_SPANSION) {
          // spansion uses MSB of bank register
          CSASSERT();
          SPI.transfer16(0x1780); // bank register write
          CSRELEASE();
          USBserial.println("spansion uses MSB of bank register");
      } else {
          // micron & winbond & macronix use command
          CSASSERT();
          SPI.transfer(0x06); // write enable
          CSRELEASE();
          delayMicroseconds(1);
          CSASSERT();
          SPI.transfer(0xB7); // enter 4 byte addr mode
          CSRELEASE();
          USBserial.println("micron & winbond & macronix use command");
      }
      SPI.endTransaction();
      if (id[0] == ID0_MICRON) f |= FLAG_MULTI_DIE;
  }
  if (id[0] == ID0_SPANSION) {
      // Spansion has separate suspend commands
      USBserial.println("Spansion has separate suspend commands");
      f |= FLAG_DIFF_SUSPEND;
      if (size >= 67108864) {
          USBserial.println("Spansion chips >= 512 mbit use 256K sectors");
          // Spansion chips >= 512 mbit use 256K sectors
          f |= FLAG_256K_BLOCKS;
      }
  }
  if (id[0] == ID0_MICRON) {
      // Micron requires busy checks with a different command
      f |= FLAG_STATUS_CMD70; // TODO: all or just multi-die chips?
      USBserial.println("Micron requires busy checks with a different command");
  }
  flags = f;
  readID(id);

  USBserial.println(id2chip((const unsigned char *)id));
  USBserial.println("init End");
  return true;
}
/**
 *
 * @param id
 * @return
 */
const char * SerialFlashChip::id2chip(const unsigned char *id)
{
  if (id[0] == 0xEF) {// Winbond
      if (id[1] == 0x40) {
          if (id[2] == 0x14) return "W25Q80BV";
          if (id[2] == 0x17) return "W25Q64FV";
          if (id[2] == 0x18) return "W25Q128FV";
          if (id[2] == 0x19) return "W25Q256FV";
      }
  }
  if (id[0] == 0x01) {// Spansion
      if (id[1] == 0x02) {
          if (id[2] == 0x16) return "S25FL064A";
          if (id[2] == 0x19) return "S25FL256S";
          if (id[2] == 0x20) return "S25FL512S";
      }
      if (id[1] == 0x20) {
          if (id[2] == 0x18) return "S25FL127S";
      }
  }
  if (id[0] == 0xC2) {// Macronix
      if (id[1] == 0x20) {
          if (id[2] == 0x18) return "MX25L12805D";
      }
  }
  if (id[0] == 0x20) {// Micron
      if (id[1] == 0xBA) {
          if (id[2] == 0x20) return "N25Q512A";
          if (id[2] == 0x21) return "N25Q00AA";
      }
      if (id[1] == 0xBB) {
          if (id[2] == 0x22) return "MT25QL02GC";
      }
  }
  if (id[0] == 0xBF) {// SST
      if (id[1] == 0x25) {
          if (id[2] == 0x02) return "SST25WF010";
          if (id[2] == 0x03) return "SST25WF020";
          if (id[2] == 0x04) return "SST25WF040";
          if (id[2] == 0x41) return "SST25VF016B";
          if (id[2] == 0x4A) return "SST25VF032";
      }
      if (id[1] == 0x25) {
          if (id[2] == 0x01) return "SST26VF016";
          if (id[2] == 0x02) return "SST26VF032";
          if (id[2] == 0x43) return "SST26VF064";
      }
  }
  return "(unknown chip)";
}
/**
 *
 */
void SerialFlashChip::sleep()
{
  if (busy) wait();
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  SPI.transfer(0xB9); // Deep power down command
  CSRELEASE();
}
/**
 *
 */
void SerialFlashChip::wakeup()
{
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  SPI.transfer(0xAB); // Wake up from deep power down command
  CSRELEASE();
}
/**
 *
 * @param buf
 */
void SerialFlashChip::readID(uint8_t *buf)
{
  if (busy) wait();
  SPI.beginTransaction(SPICONFIG);
  CSASSERT();
  SPI.transfer(0x9F);
  buf[0] = SPI.transfer(0); // manufacturer ID
  buf[1] = SPI.transfer(0); // memory type
  buf[2] = SPI.transfer(0); // capacity
  CSRELEASE();
  SPI.endTransaction();
  //Serial.printf("ID: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
}
/**
 *
 * @param id
 * @return
 */
uint32_t SerialFlashChip::capacity(const uint8_t *id)
{
  uint32_t n = 1048576; // unknown chips, default to 1 MByte

  if (id[2] >= 16 && id[2] <= 31) {
      n = 1ul << id[2];
  } else
    if (id[2] >= 32 && id[2] <= 37) {
        n = 1ul << (id[2] - 6);
    }
  //  USBserial.print("capacity");
  //  USBserial.println(n);
  return n;
}
/**
 *
 * @return
 */
uint32_t SerialFlashChip::blockSize()
{
  // Spansion chips >= 512 mbit use 256K sectors
  if (flags & FLAG_256K_BLOCKS) return 262144;
  // everything else seems to have 64K sectors
  return 65536;
}


SerialFlashChip SerialFlash;
