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
 * @file libmaple/stm32f1/include/series/flash.h
 * @brief STM32F1 Flash header.
 *
 * Provides register map, base pointer, and register bit definitions
 * for the Flash controller on the STM32F1 line, along with
 * series-specific configuration values.
 */

#ifndef _LIBMAPLE_STM32F1_FLASH_H_
#define _LIBMAPLE_STM32F1_FLASH_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/libmaple_types.h>

/*
 * Register map
 */

/** @brief STM32F1 Flash register map type */
typedef struct flash_reg_map {
    __io uint32 ACR;            /**< Access control register */
    __io uint32 KEYR;           /**< Key register */
    __io uint32 OPTKEYR;        /**< OPTKEY register */
    __io uint32 SR;             /**< Status register */
    __io uint32 CR;             /**< Control register */
    __io uint32 AR;             /**< Address register */
    __io uint32 OBR;            /**< Option byte register */
    __io uint32 WRPR;           /**< Write protection register */
} flash_reg_map;

#define MAPLE_FLASH_BASE                      ((struct flash_reg_map*)0x40022000)

/**
  * @brief Option Bytes Registers
  */

typedef struct
{
  __io uint16_t RDP;
  __io uint16_t USER;
  __io uint16_t Data0;
  __io uint16_t Data1;
  __io uint16_t WRP0;
  __io uint16_t WRP1;
  __io uint16_t WRP2;
  __io uint16_t WRP3;
} ob_TypeDef;

#define OB_BASE               ((uint32_t)0x1FFFF800)    /*!< Flash Option Bytes base address */
#define MAPLE_OB                  ((ob_TypeDef *) OB_BASE)

/*
 * Register bit definitions
 */
#define  MAPLE_FLASH_ACR_LATENCY                   ((uint8_t)0x03)               /*!< LATENCY[2:0] bits (Latency) */
#define  MAPLE_FLASH_ACR_LATENCY_0                 ((uint8_t)0x00)               /*!< Bit 0 */
#define  MAPLE_FLASH_ACR_LATENCY_1                 ((uint8_t)0x01)               /*!< Bit 0 */
#define  MAPLE_FLASH_ACR_LATENCY_2                 ((uint8_t)0x02)               /*!< Bit 1 */
/* Access control register */

#define MAPLE_FLASHACR_PRFTBS_BIT            5
#define MAPLE_FLASHACR_PRFTBE_BIT            4
#define MAPLE_FLASHACR_HLFCYA_BIT            3

#define MAPLE_FLASHACR_PRFTBS                (1U << MAPLE_FLASHACR_PRFTBS_BIT)
#define MAPLE_FLASHACR_PRFTBE                (1U << MAPLE_FLASHACR_PRFTBE_BIT)
#define MAPLE_FLASHACR_HLFCYA                (1U << MAPLE_FLASHACR_HLFCYA_BIT)
#define MAPLE_FLASHACR_LATENCY               0x7

/* Status register */

#define MAPLE_FLASHSR_EOP_BIT                5
#define MAPLE_FLASHSR_WRPRTERR_BIT           4
#define MAPLE_FLASHSR_PGERR_BIT              2
#define MAPLE_FLASHSR_BSY_BIT                0

#define MAPLE_FLASHSR_EOP                    (1U << MAPLE_FLASHSR_EOP_BIT)
#define MAPLE_FLASHSR_WRPRTERR               (1U << MAPLE_FLASHSR_WRPRTERR_BIT)
#define MAPLE_FLASHSR_PGERR                  (1U << MAPLE_FLASHSR_PGERR_BIT)
#define MAPLE_FLASHSR_BSY                    (1U << MAPLE_FLASHSR_BSY_BIT)

/* Control register */

#define MAPLE_FLASHCR_EOPIE_BIT              12
#define MAPLE_FLASHCR_ERRIE_BIT              10
#define MAPLE_FLASHCR_OPTWRE_BIT             9
#define MAPLE_FLASHCR_LOCK_BIT               7
#define MAPLE_FLASHCR_STRT_BIT               6
#define MAPLE_FLASHCR_OPTER_BIT              5
#define MAPLE_FLASHCR_OPTPG_BIT              4
#define MAPLE_FLASHCR_MER_BIT                2
#define MAPLE_FLASHCR_PER_BIT                1
#define MAPLE_FLASHCR_PG_BIT                 0

#define MAPLE_FLASHCR_EOPIE                  (1U << MAPLE_FLASHCR_EOPIE_BIT)
#define MAPLE_FLASHCR_ERRIE                  (1U << MAPLE_FLASHCR_ERRIE_BIT)
#define MAPLE_FLASHCR_OPTWRE                 (1U << MAPLE_FLASHCR_OPTWRE_BIT)
#define MAPLE_FLASHCR_LOCK                   (1U << MAPLE_FLASHCR_LOCK_BIT)
#define MAPLE_FLASHCR_STRT                   (1U << MAPLE_FLASHCR_STRT_BIT)
#define MAPLE_FLASHCR_OPTER                  (1U << MAPLE_FLASHCR_OPTER_BIT)
#define MAPLE_FLASHCR_OPTPG                  (1U << MAPLE_FLASHCR_OPTPG_BIT)
#define MAPLE_FLASHCR_MER                    (1U << MAPLE_FLASHCR_MER_BIT)
#define MAPLE_FLASHCR_PER                    (1U << MAPLE_FLASHCR_PER_BIT)
#define MAPLE_FLASHCR_PG                     (1U << MAPLE_FLASHCR_PG_BIT)

/* Option byte register */

#define MAPLE_FLASHOBR_nRST_STDBY_BIT        4
#define MAPLE_FLASHOBR_nRST_STOP_BIT         3
#define MAPLE_FLASHOBR_WDG_SW_BIT            2
#define MAPLE_FLASHOBR_RDPRT_BIT             1
#define MAPLE_FLASHOBR_OPTERR_BIT            0

#define MAPLE_FLASHOBR_DATA1                 (0xFF << 18)
#define MAPLE_FLASHOBR_DATA0                 (0xFF << 10)
#define MAPLE_FLASHOBR_USER                  0x3FF
#define MAPLE_FLASHOBR_nRST_STDBY            (1U << MAPLE_FLASHOBR_nRST_STDBY_BIT)
#define MAPLE_FLASHOBR_nRST_STOP             (1U << MAPLE_FLASHOBR_nRST_STOP_BIT)
#define MAPLE_FLASHOBR_WDG_SW                (1U << MAPLE_FLASHOBR_WDG_SW_BIT)
#define MAPLE_FLASHOBR_RDPRT                 (1U << MAPLE_FLASHOBR_RDPRT_BIT)
#define MAPLE_FLASHOBR_OPTERR                (1U << MAPLE_FLASHOBR_OPTERR_BIT)
/*
 * Series-specific configuration values.
 */

#define MAPLE_FLASHSAFE_WAIT_STATES          MAPLE_FLASHWAIT_STATE_2

/* Flash memory features available via ACR */
enum {
    MAPLE_FLASHPREFETCH   = 0x10,
    MAPLE_FLASHHALF_CYCLE = 0x8,
    MAPLE_FLASHICACHE     = 0x0,     /* Not available on STM32F1 */
    MAPLE_FLASHDCACHE     = 0x0,     /* Not available on STM32F1 */
};



typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;


/** @defgroup FLASH_Flags
  * @{
  */
#ifdef STM32F10X_XL
#define FLASH_FLAG_BANK2_BSY                 ((uint32_t)0x80000001)  /*!< FLASH BANK2 Busy flag */
#define FLASH_FLAG_BANK2_EOP                 ((uint32_t)0x80000020)  /*!< FLASH BANK2 End of Operation flag */
#define FLASH_FLAG_BANK2_PGERR               ((uint32_t)0x80000004)  /*!< FLASH BANK2 Program error flag */
#define FLASH_FLAG_BANK2_WRPRTERR            ((uint32_t)0x80000010)  /*!< FLASH BANK2 Write protected error flag */

#define FLASH_FLAG_BANK1_BSY                 FLASH_FLAG_BSY       /*!< FLASH BANK1 Busy flag*/
#define FLASH_FLAG_BANK1_EOP                 FLASH_FLAG_EOP       /*!< FLASH BANK1 End of Operation flag */
#define FLASH_FLAG_BANK1_PGERR               FLASH_FLAG_PGERR     /*!< FLASH BANK1 Program error flag */
#define FLASH_FLAG_BANK1_WRPRTERR            FLASH_FLAG_WRPRTERR  /*!< FLASH BANK1 Write protected error flag */

#define FLASH_FLAG_BSY                 ((uint32_t)0x00000001)  /*!< FLASH Busy flag */
#define FLASH_FLAG_EOP                 ((uint32_t)0x00000020)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_PGERR               ((uint32_t)0x00000004)  /*!< FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR            ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_OPTERR              ((uint32_t)0x00000001)  /*!< FLASH Option Byte error flag */

#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0x7FFFFFCA) == 0x00000000) && ((FLAG) != 0x00000000))
#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_BSY) || ((FLAG) == FLASH_FLAG_EOP) || \
                                  ((FLAG) == FLASH_FLAG_PGERR) || ((FLAG) == FLASH_FLAG_WRPRTERR) || \
                                  ((FLAG) == FLASH_FLAG_OPTERR)|| \
                                  ((FLAG) == FLASH_FLAG_BANK1_BSY) || ((FLAG) == FLASH_FLAG_BANK1_EOP) || \
                                  ((FLAG) == FLASH_FLAG_BANK1_PGERR) || ((FLAG) == FLASH_FLAG_BANK1_WRPRTERR) || \
                                  ((FLAG) == FLASH_FLAG_BANK2_BSY) || ((FLAG) == FLASH_FLAG_BANK2_EOP) || \
                                  ((FLAG) == FLASH_FLAG_BANK2_PGERR) || ((FLAG) == FLASH_FLAG_BANK2_WRPRTERR))
#else
#define FLASH_FLAG_BSY                 ((uint32_t)0x00000001)  /*!< FLASH Busy flag */
#define FLASH_FLAG_EOP                 ((uint32_t)0x00000020)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_PGERR               ((uint32_t)0x00000004)  /*!< FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR            ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_OPTERR              ((uint32_t)0x00000001)  /*!< FLASH Option Byte error flag */

#define FLASH_FLAG_BANK1_BSY                 FLASH_FLAG_BSY       /*!< FLASH BANK1 Busy flag*/
#define FLASH_FLAG_BANK1_EOP                 FLASH_FLAG_EOP       /*!< FLASH BANK1 End of Operation flag */
#define FLASH_FLAG_BANK1_PGERR               FLASH_FLAG_PGERR     /*!< FLASH BANK1 Program error flag */
#define FLASH_FLAG_BANK1_WRPRTERR            FLASH_FLAG_WRPRTERR  /*!< FLASH BANK1 Write protected error flag */

#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFCA) == 0x00000000) && ((FLAG) != 0x00000000))
#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_BSY) || ((FLAG) == FLASH_FLAG_EOP) || \
                                  ((FLAG) == FLASH_FLAG_PGERR) || ((FLAG) == FLASH_FLAG_WRPRTERR) || \
                  ((FLAG) == FLASH_FLAG_BANK1_BSY) || ((FLAG) == FLASH_FLAG_BANK1_EOP) || \

#endif
#ifdef __cplusplus
}
#endif

#endif
