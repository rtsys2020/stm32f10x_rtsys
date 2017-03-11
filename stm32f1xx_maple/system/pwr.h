/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2012 LeafLabs, LLC.
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
 * @file libmaple/stm32f1/include/series/pwr.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief STM32F1 Power control (PWR) support.
 */

#ifndef _LIBMAPLE_STM32F1_PWR_H_
#define _LIBMAPLE_STM32F1_PWR_H_

/*
 * Register bit definitions
 */

/* Control register */

/* PVD level selection */
#define PWR_CR_PLS_2_2V (0x0 << 5)
#define PWR_CR_PLS_2_3V (0x1 << 5)
#define PWR_CR_PLS_2_4V (0x2 << 5)
#define PWR_CR_PLS_2_5V (0x3 << 5)
#define PWR_CR_PLS_2_6V (0x4 << 5)
#define PWR_CR_PLS_2_7V (0x5 << 5)
#define PWR_CR_PLS_2_8V (0x6 << 5)
#define PWR_CR_PLS_2_9V (0x7 << 5)

#define PWR_PVDLevel_2V2          ((uint32_t)0x00000000)
#define PWR_PVDLevel_2V3          ((uint32_t)0x00000020)
#define PWR_PVDLevel_2V4          ((uint32_t)0x00000040)
#define PWR_PVDLevel_2V5          ((uint32_t)0x00000060)
#define PWR_PVDLevel_2V6          ((uint32_t)0x00000080)
#define PWR_PVDLevel_2V7          ((uint32_t)0x000000A0)
#define PWR_PVDLevel_2V8          ((uint32_t)0x000000C0)
#define PWR_PVDLevel_2V9          ((uint32_t)0x000000E0)


//#define  SCB_SCR_SLEEPONEXIT                 ((uint8_t)0x02)               /*!< Sleep on exit bit */
//#define  SCB_SCR_SLEEPDEEP                   ((uint8_t)0x04)               /*!< Sleep deep bit */
//#define  SCB_SCR_SEVONPEND                   ((uint8_t)0x10)               /*!< Wake up from WFE */
#define PWR_STOPEntry_WFI         ((uint8_t)0x01)
#define PWR_STOPEntry_WFE         ((uint8_t)0x02)

/* CR register bit mask */
#define CR_DS_MASK               ((uint32_t)0xFFFFFFFC)
#define CR_PLS_MASK              ((uint32_t)0xFFFFFF1F)

#define PWR_FLAG_WU               ((uint32_t)0x00000001)
#define PWR_FLAG_SB               ((uint32_t)0x00000002)
#define PWR_FLAG_PVDO             ((uint32_t)0x00000004)


/* Alias word address of DBP bit */
#define CR_OFFSET                (0x7000 )
#define DBP_BitNumber            0x08
#define CR_DBP_BB                (0x42000000 + (CR_OFFSET * 32) + (DBP_BitNumber * 4))
/* Alias word address of PVDE bit */
#define PVDE_BitNumber           0x04
#define CR_PVDE_BB               (0x42000000 + (CR_OFFSET * 32) + (PVDE_BitNumber * 4))

/* --- CSR Register ---*/

/* Alias word address of EWUP bit */
#define PWR_OFFSET 0x7000
#define CSR_OFFSET               (PWR_OFFSET + 0x04)
#define EWUP_BitNumber           0x08
#define CSR_EWUP_BB              (0x42000000 + (CSR_OFFSET * 32) + (EWUP_BitNumber * 4))

#endif
