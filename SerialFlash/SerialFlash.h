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

#ifndef SerialFlash_h_
#define SerialFlash_h_

#include <Arduino.h>
#include <SPI.h>

#define SPI SPIClass1

#define CSASSERT()  DIRECT_WRITE_LOW(cspin_basereg, cspin_bitmask)
#define CSRELEASE() DIRECT_WRITE_HIGH(cspin_basereg, cspin_bitmask)
#define SPICONFIG   SPISettings(4000000, MSBFIRST, SPI_MODE0)



#define FLAG_32BIT_ADDR   0x01  // larger than 16 MByte address
#define FLAG_STATUS_CMD70 0x02  // requires special busy flag check
#define FLAG_DIFF_SUSPEND 0x04  // uses 2 different suspend commands
#define FLAG_MULTI_DIE    0x08  // multiple die, don't read cross 32M barrier
#define FLAG_256K_BLOCKS  0x10  // has 256K erase blocks
#define FLAG_DIE_MASK   0xC0  // top 2 bits count during multi-die erase

/*
Chip    Uniform Sector Erase
    20/21 52  D8/DC
    ----- --  -----
W25Q64CV  4 32  64
W25Q128FV 4 32  64
S25FL127S     64
N25Q512A  4   64
N25Q00AA  4   64
S25FL512S     256
SST26VF032  4
 */



//      size  sector      busy  pgm/erase chip
// Part     Mbyte kbyte ID bytes  cmd suspend   erase
// ----     ----  ----- --------  --- -------   -----
// Winbond W25Q64CV 8 64  EF 40 17
// Winbond W25Q128FV  16  64  EF 40 18  05  single    60 & C7
// Winbond W25Q256FV  32  64  EF 40 19
// Spansion S25FL064A 8 ? 01 02 16
// Spansion S25FL127S 16  64  01 20 18  05
// Spansion S25FL128P 16  64  01 20 18
// Spansion S25FL256S 32  64  01 02 19  05      60 & C7
// Spansion S25FL512S 64  256 01 02 20
// Macronix MX25L12805D 16  ? C2 20 18
// Macronix MX66L51235F 64    C2 20 1A
// Numonyx M25P128  16  ? 20 20 18
// Micron M25P80  1 ? 20 20 14
// Micron N25Q128A  16  64  20 BA 18
// Micron N25Q512A  64  ? 20 BA 20  70  single    C4 x2
// Micron N25Q00AA  128 64  20 BA 21    single    C4 x4
// Micron MT25QL02GC  256 64  20 BA 22  70      C4 x2
// SST SST25WF010 1/8 ? BF 25 02
// SST SST25WF020 1/4 ? BF 25 03
// SST SST25WF040 1/2 ? BF 25 04
// SST SST25VF016B  1 ? BF 25 41
// SST26VF016     ? BF 26 01
// SST26VF032     ? BF 26 02
// SST25VF032   4 64  BF 25 4A
// SST26VF064   8 ? BF 26 43
// LE25U40CMC   1/2 64  62 06 13

#define ID0_WINBOND 0xEF
#define ID0_SPANSION  0x01
#define ID0_MICRON  0x20
#define ID0_MACRONIX  0xC2
#define ID0_SST   0xBF


class SerialFlashFile;

class SerialFlashChip
{
public:
	static bool begin(uint8_t pin = 6);
	static uint32_t capacity(const uint8_t *id);
	static uint32_t blockSize();
	static void sleep();
	static void wakeup();
	static void readID(uint8_t *buf);
	static bool read(uint32_t addr, void *buf, uint32_t len);
	static bool ready();
	static void wait();
	static bool write(uint32_t addr, const void *buf, uint32_t len);
	static void eraseAll();
	static void eraseBlock(uint32_t addr);
	static bool eraseSector(uint32_t addr);
	static const char * id2chip(const unsigned char *id);
	static SerialFlashFile open(const char *filename);
	static bool create(const char *filename, uint32_t length, uint32_t align = 0);
	static bool createErasable(const char *filename, uint32_t length) {
		return create(filename, length, blockSize());
	}
	static bool exists(const char *filename);
	static bool remove(const char *filename);
	static bool remove(SerialFlashFile &file);
	static void opendir() { dirindex = 0; }
	static bool readdir(char *filename, uint32_t strsize, uint32_t &filesize);
	bool writePage(uint32_t addr, const void *buf,uint8_t len);
	uint32_t readReg8(uint8_t reg);
	uint32_t readReg16(uint8_t reg);
	uint32_t readReg32(uint8_t reg);
	void writeEn(void);
private:
	static uint16_t dirindex; // current position for readdir()
	static uint8_t flags;	// chip features
	static uint8_t busy;	// 0 = ready
				// 1 = suspendable program operation
				// 2 = suspendable erase operation
				// 3 = busy for realz!!
};

extern SerialFlashChip SerialFlash;


class SerialFlashFile
{
public:
	SerialFlashFile() : address(0) {
	  length=0;
	  dirindex=0;
	  offset=0;
	}
	operator bool() {
		if (address > 0) return true;
		return false;
	}
	uint32_t read(void *buf, uint32_t rdlen) {
		if (offset + rdlen > length) {
			if (offset >= length) return 0;
			rdlen = length - offset;
		}
		SerialFlash.read(address + offset, buf, rdlen);
		offset += rdlen;
		return rdlen;
	}
	uint32_t write(const void *buf, uint32_t wrlen) {
		if (offset + wrlen > length) {
			if (offset >= length) return 0;
			wrlen = length - offset;
		}
		SerialFlash.write(address + offset, buf, wrlen);
		offset += wrlen;
		return wrlen;
	}
	void seek(uint32_t n) {
		offset = n;
	}
	uint32_t position() {
		return offset;
	}
	uint32_t size() {
		return length;
	}
	uint32_t available() {
		if (offset >= length) return 0;
		return length - offset;
	}
	void erase();
	void flush() {
	}
	void close() {
	}
	uint32_t getFlashAddress() {
		return address;
	}
protected:
	friend class SerialFlashChip;
	uint32_t address;  // where this file's data begins in the Flash, or zero
	uint32_t length;   // total length of the data in the Flash chip
	uint32_t offset; // current read/write offset in the file
	uint16_t dirindex;
};


#endif
