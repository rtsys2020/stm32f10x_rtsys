/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file libmaple/rcc.c
 * @brief Portable RCC routines.
 */

#include <libmaple/rcc.h>

#include "libmaple/rcc_private.h"

/**
 * @brief Get a peripheral's clock domain
 * @param id Clock ID of the peripheral whose clock domain to return
 * @return Clock source for the given clock ID
 */
rcc_clk_domain rcc_dev_clk(rcc_clk_id id) {
    return rcc_dev_table[id].clk_domain;
}

/**
 * @brief Switch the clock used as the source of the system clock.
 *
 * After switching the source, this function blocks until the new
 * clock source is in use.
 *
 * @param sysclk_src New system clock source.
 * @see rcc_sysclk_src
 */
void rcc_switch_sysclk(rcc_sysclk_src sysclk_src) {
    uint32 cfgr = MAPLE_RCC_BASE->CFGR;
    cfgr &= ~MAPLE_RCC_CFGR_SW;
    cfgr |= sysclk_src;

    /* Switch SYSCLK source. */
    MAPLE_RCC_BASE->CFGR = cfgr;

    /* Wait for new source to come into use. */
    while ((MAPLE_RCC_BASE->CFGR & MAPLE_RCC_CFGR_SWS) != (sysclk_src << 2))
        ;
}

/*
 * Turning clocks off and on, querying their status.
 */

/* IMPORTANT NOTE FOR IMPLEMENTORS:
 *
 * libmaple assumes that enum rcc_clk enumerators are two-byte
 * values, stored in a uint16, in the following way:
 *
 *  - The high-order byte is the byte offset (from RCC_BASE) of the register
 *    to touch when turning on or off the given clock.
 *
 *  - The low-order byte is the bit in that register that turns the
 *    clock on or off.
 *
 * Example for STM32F1: Turning on the high-speed external clock (HSE)
 * involves setting HSEON, bit 16, of RCC_CR. The high-order byte is
 * then offsetof(struct rcc_reg_map, CR) = 0, and the low-order byte
 * is 16.
 *
 * The corresponding value of RCC_CLK_HSE is thus (0 << 8) | 16 = 16.
 *
 * On all known STM32 series, this encoding has the property that
 * adding one to the low byte also gives the bit to check to determine
 * if the clock is ready. For example, on STM32F1, RCC_CR_HSERDY is
 * bit 17. If that's not the case on your series, rcc_is_clk_ready()
 * won't work for you. */

/* Returns the RCC register which controls the clock source. */
static inline __io uint32* rcc_clk_reg(rcc_clk clock) {
    return (__io uint32*)((__io uint8*)MAPLE_RCC_BASE + (clock >> 8));
}

/* Returns a mask in rcc_clk_reg(clock) to be used for turning the
 * clock on and off */
static inline uint32 rcc_clk_on_mask(rcc_clk clock) {
    return 1 << (clock & 0xFF);
}

/* Returns a mask in rcc_clk_reg(clock) to be used when checking the
 * readiness of the clock. */
static inline uint32 rcc_clk_ready_mask(rcc_clk clock) {
    return rcc_clk_on_mask(clock) << 1;
}

/**
 * @brief Turn on a clock source.
 *
 * After this routine exits, callers should ensure that the clock
 * source is ready by waiting until rcc_is_clk_ready(clock) returns
 * true.
 *
 * @param clock Clock to turn on.
 * @see rcc_turn_off_clk()
 * @see rcc_is_clk_ready()
 */
void rcc_turn_on_clk(rcc_clk clock) {
    *rcc_clk_reg(clock) |= rcc_clk_on_mask(clock);
}

/**
 * @brief Turn off a clock source.
 *
 * In certain configurations, certain clock sources cannot be safely
 * turned off. (For example, the main PLL on STM32F1 devices cannot be
 * turned off if it has been selected as the SYSCLK source). Consult
 * the reference material for your MCU to ensure it is safe to call
 * this function.
 *
 * @param clock Clock to turn off.
 * @see rcc_turn_on_clk()
 * @see rcc_is_clk_ready()
 */
void rcc_turn_off_clk(rcc_clk clock) {
    *rcc_clk_reg(clock) &= ~rcc_clk_on_mask(clock);
}

/**
 * @brief Check if a clock is on.
 * @param clock Clock to check.
 * @return 1 if the clock is on, 0 if the clock is off.
 */
int rcc_is_clk_on(rcc_clk clock) {
    return !!(*rcc_clk_reg(clock) & rcc_clk_on_mask(clock));
}

/**
 * @brief Check if a clock source is ready.
 *
 * In general, it is not safe to rely on a clock source unless this
 * function returns nonzero. Also note that this function may return
 * nonzero for a short period of time after a clock has been turned
 * off. Consult the reference material for your MCU for more details.
 *
 * @param clock Clock whose readiness to check for.
 * @return Nonzero if the clock is ready, zero otherwise.
 * @see rcc_turn_on_clk()
 * @see rcc_turn_off_clk()
 */
int rcc_is_clk_ready(rcc_clk clock) {
    return (int)(*rcc_clk_reg(clock) & rcc_clk_ready_mask(clock));
}

//#define BDCR_ADDRESS              (0x40021020)
/**
  * @brief  Configures the External Low Speed oscillator (LSE).
  * @param  RCC_LSE: specifies the new state of the LSE.
  *   This parameter can be one of the following values:
  *     @arg RCC_LSE_OFF: LSE oscillator OFF
  *     @arg RCC_LSE_ON: LSE oscillator ON
  *     @arg RCC_LSE_Bypass: LSE oscillator bypassed with external clock
  * @retval None
  */
void rcc_lseconfig(uint8_t RCC_LSE)
{

  /* Check the parameters */

  /* Reset LSEON and LSEBYP bits before configuring the LSE ------------------*/
  /* Reset LSEON bit */
//  *(__io uint8_t *) BDCR_ADDRESS = RCC_LSE_OFF;
  MAPLE_RCC_BASE->BDCR = MAPLE_RCC_LSE_OFF;
  /* Reset LSEBYP bit */
//  *(__io uint8_t *) BDCR_ADDRESS = RCC_LSE_OFF;
  MAPLE_RCC_BASE->BDCR = MAPLE_RCC_LSE_OFF;
  /* Configure LSE (RCC_LSE_OFF is already covered by the code section above) */
  switch(RCC_LSE)
  {
    case MAPLE_RCC_LSE_ON:
      /* Set LSEON bit */
      MAPLE_RCC_BASE->BDCR = MAPLE_RCC_LSE_ON;
      break;

    case MAPLE_RCC_LSE_Bypass:
      /* Set LSEBYP and LSEON bits */
      MAPLE_RCC_BASE->BDCR = MAPLE_RCC_LSE_Bypass | MAPLE_RCC_LSE_ON;
      break;

    default:
      break;
  }
}

/**
  * @brief  Checks whether the specified RCC flag is set or not.
  * @param  RCC_FLAG: specifies the flag to check.
  *
  *   For @b STM32_Connectivity_line_devices, this parameter can be one of the
  *   following values:
  *     @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready
  *     @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
  *     @arg RCC_FLAG_PLLRDY: PLL clock ready
  *     @arg RCC_FLAG_PLL2RDY: PLL2 clock ready
  *     @arg RCC_FLAG_PLL3RDY: PLL3 clock ready
  *     @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
  *     @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
  *     @arg RCC_FLAG_PINRST: Pin reset
  *     @arg RCC_FLAG_PORRST: POR/PDR reset
  *     @arg RCC_FLAG_SFTRST: Software reset
  *     @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
  *     @arg RCC_FLAG_WWDGRST: Window Watchdog reset
  *     @arg RCC_FLAG_LPWRRST: Low Power reset
  *
  *   For @b other_STM32_devices, this parameter can be one of the following values:
  *     @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready
  *     @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
  *     @arg RCC_FLAG_PLLRDY: PLL clock ready
  *     @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
  *     @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
  *     @arg RCC_FLAG_PINRST: Pin reset
  *     @arg RCC_FLAG_PORRST: POR/PDR reset
  *     @arg RCC_FLAG_SFTRST: Software reset
  *     @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
  *     @arg RCC_FLAG_WWDGRST: Window Watchdog reset
  *     @arg RCC_FLAG_LPWRRST: Low Power reset
  *
  * @retval The new state of RCC_FLAG (SET or RESET).
  */
uint8_t rcc_getflagstatus(uint8_t RCC_FLAG)
{
  uint32_t tmp = 0;
  uint32_t statusreg = 0;
  uint8_t bitstatus = 0;
  /* Check the parameters */
//  assert_param(IS_RCC_FLAG(RCC_FLAG));

  /* Get the RCC register index */
  tmp = RCC_FLAG >> 5;
  if (tmp == 1)               /* The flag to check is in CR register */
  {
    statusreg = MAPLE_RCC_BASE->CR;
  }
  else if (tmp == 2)          /* The flag to check is in BDCR register */
  {
    statusreg = MAPLE_RCC_BASE->BDCR;
  }
  else                       /* The flag to check is in CSR register */
  {
    statusreg = MAPLE_RCC_BASE->CSR;
  }

  /* Get the flag position */
  tmp = RCC_FLAG & 0x1F;
  if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)0)
  {
    bitstatus = 1;
  }
  else
  {
    bitstatus = 0;
  }

  /* Return the flag status */
  return bitstatus;
}


void rcc_set_pllxtpre(uint32_t pllxtpre)
{
  uint32_t reg32;

  reg32 = MAPLE_RCC_BASE->CFGR;
  reg32 &= ~(1 << 17);
  MAPLE_RCC_BASE->CFGR = (reg32 | (pllxtpre << 17));
}
