/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/include/libmaple/flash.h
 * @brief Flash support.
 */

#ifndef _LIBMAPLE_FLASH_H_
#define _LIBMAPLE_FLASH_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/libmaple_types.h>

#define MAPLE_FLASH_WAIT_STATE_0              0x0
#define MAPLE_FLASH_WAIT_STATE_1              0x1
#define MAPLE_FLASH_WAIT_STATE_2              0x2
#define MAPLE_FLASH_WAIT_STATE_3              0x3
#define MAPLE_FLASH_WAIT_STATE_4              0x4
#define MAPLE_FLASH_WAIT_STATE_5              0x5
#define MAPLE_FLASH_WAIT_STATE_6              0x6
#define MAPLE_FLASH_WAIT_STATE_7              0x7

/* The series header must define:
 *
 * - FLASH_SAFE_WAIT_STATES, the smallest number of wait states that
 *   it is safe to use when SYSCLK is at its fastest documented rate
 *   and the MCU is powered at 3.3V (i.e. this doesn't consider
 *   overclocking or low voltage operation).
 *
 * - The following bit flags, for flash_enable_features():
 *
 *       -- FLASH_PREFETCH: prefetcher
 *       -- FLASH_ICACHE: instruction cache
 *       -- FLASH_DCACHE: data cache
 *
 *   See that function's Doxygen for more restrictions.
 */
 /* Roger clark. Replaced with line below #include <series/flash.h>*/
#include "system/flash.h"

#ifdef __DOXYGEN__
/** Flash register map base pointer. */
#define MAPLE_FLASH_BASE
#endif

/*
 * Flash routines
 */

void flash_set_latency(uint32 wait_states);

/**
 * @brief Enable Flash memory features
 *
 * If the target MCU doesn't provide a feature (e.g. instruction and
 * data caches on the STM32F1), the flag will be ignored. This allows
 * using these flags unconditionally, with the desired effect taking
 * place on targets that support them.
 *
 * @param feature_flags Bitwise OR of the following:
 *                      FLASH_PREFETCH (turns on prefetcher),
 *                      FLASH_ICACHE (turns on instruction cache),
 *                      FLASH_DCACHE (turns on data cache).
 */
static inline void flash_enable_features(uint32 feature_flags) {
  MAPLE_FLASH_BASE->ACR |= feature_flags;
}

/**
 * @brief Deprecated. Use flash_enable_features(FLASH_PREFETCH) instead.
 */
static inline void flash_enable_prefetch(void) {
    flash_enable_features(MAPLE_FLASHPREFETCH);
}

void flash_halfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void flash_prefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void flash_unlock(void);
void flash_unlockBank1(void);
void flash_lock(void);
void flash_lockBank1(void);
FLASH_Status flash_ErasePage(uint32_t Page_Address);
FLASH_Status flash_EraseAllPages(void);
FLASH_Status flash_EraseAllBank1Pages(void);
FLASH_Status flash_EraseOptionBytes(void);
FLASH_Status flash_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status flash_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status flash_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status flash_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status flash_ReadOutProtection(uint8_t NewState);
FLASH_Status flash_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t flash_GetWriteProtectionOptionByte(void);
uint8_t flash_GetPrefetchBufferStatus(void);
void flash_ITConfig(uint32_t FLASH_IT, uint8_t NewState);
uint8_t flash_GetReadOutProtectionStatus(void);
void flash_ClearFlag(uint32_t FLASH_FLAG);
uint8_t flash_GetFlagStatus(uint32_t FLASH_FLAG);
uint8_t flash_GetStatus(void);
uint8_t flash_GetBank1Status(void);
uint8_t flash_WaitForLastOperation(uint32_t Timeout);
uint8_t flash_WaitForLastBank1Operation(uint32_t Timeout);
uint8_t flash_WaitForLastBank2Operation(uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif
