/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs, LLC.
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
 * @file libmaple/include/libmaple/pwr.h
 * @brief Power control (PWR).
 */

#ifndef _LIBMAPLE_PWR_H_
#define _LIBMAPLE_PWR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <libmaple/libmaple.h>
 /* Roger clark. Replaced with line below #include <series/pwr.h>*/
#include "system/pwr.h"


/** Power interface register map. */
typedef struct pwr_reg_map {
    __io uint32 CR;      /**< Control register */
    __io uint32 CSR;     /**< Control and status register */
} pwr_reg_map;

/** Power peripheral register map base pointer. */
#define MAPLE_PWR_BASE                        ((struct pwr_reg_map*)0x40007000)




/* --- CR Register ---*/



/*
 * Register bit definitions
 */

/* Control register */

/** Disable backup domain write protection bit */
#define MAPLE_PWR_CR_DBP_BIT                  8
/** Power voltage detector enable bit */
#define MAPLE_PWR_CR_PVDE_BIT                 4
/** Clear standby flag bit */
#define MAPLE_PWR_CR_CSBF_BIT                 3
/** Clear wakeup flag bit */
#define MAPLE_PWR_CR_CWUF_BIT                 2
/** Power down deepsleep bit */
#define MAPLE_PWR_CR_PDDS_BIT                 1
/** Low-power deepsleep bit */
#define MAPLE_PWR_CR_LPDS_BIT                 0

/** Disable backup domain write protection */
#define MAPLE_PWR_CR_DBP                      (1U << MAPLE_PWR_CR_DBP_BIT)
/** Power voltage detector (PVD) level selection */
#define MAPLE_PWR_CR_PLS                      (0x7 << 5)
/** Power voltage detector enable */
#define MAPLE_PWR_CR_PVDE                     (1U << MAPLE_PWR_CR_PVDE_BIT)
/** Clear standby flag */
#define MAPLE_PWR_CR_CSBF                     (1U << MAPLE_PWR_CR_CSBF_BIT)
/** Clear wakeup flag */
#define MAPLE_PWR_CR_CWUF                     (1U << MAPLE_PWR_CR_CWUF_BIT)
/** Power down deepsleep */
#define MAPLE_PWR_CR_PDDS                     (1U << MAPLE_PWR_CR_PDDS_BIT)
/** Low-power deepsleep */
#define MAPLE_PWR_CR_LPDS                     (1U << MAPLE_PWR_CR_LPDS_BIT)

/* Control and status register */

/** Enable wakeup pin bit */
#define MAPLE_PWR_CSR_EWUP_BIT                 8
/** PVD output bit */
#define MAPLE_PWR_CSR_PVDO_BIT                 2
/** Standby flag bit */
#define MAPLE_PWR_CSR_SBF_BIT                  1
/** Wakeup flag bit */
#define MAPLE_PWR_CSR_WUF_BIT                  0

/** Enable wakeup pin */
#define MAPLE_PWR_CSR_EWUP                     (1U << MAPLE_PWR_CSR_EWUP_BIT)
/** PVD output */
#define MAPLE_PWR_CSR_PVDO                     (1U << MAPLE_PWR_CSR_PVDO_BIT)
/** Standby flag */
#define MAPLE_PWR_CSR_SBF                      (1U << MAPLE_PWR_CSR_SBF_BIT)
/** Wakeup flag */
#define MAPLE_PWR_CSR_WUF                      (1U << MAPLE_PWR_CSR_WUF_BIT)

/*
 * Convenience functions
 */

void pwr_init(void);
void pwr_stopmode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void pwr_bkp_access_cmd(uint8_t NewState);
void pwr_pvd_cmd(uint8_t NewState);

uint8_t pwr_getflag_status(uint32_t PWR_FLAG);
void pwr_clrflag(uint32_t PWR_FLAG);
void pwr_wakeup_pincmd(uint8_t NewState);
void pwr_PVDlevelset(uint32_t PWR_PVDLevel);

void pwr_stopinit(uint8_t clksrc);
void pwr_stopGo(uint32_t tim);

#ifdef __cplusplus
}
#endif

#endif
