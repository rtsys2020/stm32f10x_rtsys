/******************************************************************************
 * The MIT License
 *
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
 * @file libmaple/pwr.c
 * @brief Power control (PWR) support.
 */

#include <libmaple/pwr.h>
#include <libmaple/rcc.h>
#include <libmaple/scb.h>
#include <libmaple/rtc.h>
#include <libmaple/nvic.h>
#include <libmaple/exti.h>
/**
 * Enables the power interface clock, and resets the power device.
 */
void pwr_init(void) {
    rcc_clk_enable(RCC_PWR);
    rcc_reset_dev(RCC_PWR);
}

/**
 *
 * @param PWR_Regulator 0 regulator on during Stop mode
 *                      1 off
 * @param PWR_STOPEntry 1 WFI wait for interrupt mode  0 WFE wait for event mode
 */
/**
 Bit0 PWR->CR
  LPDS:Low-powerdeepsleep.
  This bit is set and cleared by software. It works together with the PDDS bit.
    0: Voltage regulator on during Stop mode
    1: Voltage regulator in low-power mode during Stop mode

 Bit1 PDDS:Powerdowndeepsleep.
  This bit is set and cleared by software. It works together with the LPDS bit.
    0: Enter Stop mode when the CPU enters Deepsleep. The regulator status depends on the LPDS bit.
    1: Enter Standby mode when the CPU enters Deepsleep.

    **/
void pwr_stopmode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
  uint32_t tmpreg = 0;


  /**If the application needs to disable the external
   clock before entering Stop mode, the HSEON bit must
  first be disabled and the system clock switched to HSI**/

  MAPLE_RCC_BASE->CFGR = 0x00000000;
  rcc_disable_css();
  rcc_turn_off_clk(MAPLE_RCC_CLK_PLL);
  rcc_turn_off_clk(MAPLE_RCC_CLK_HSE);

  /* Select the regulator state in STOP mode ---------------------------------*/
  tmpreg = MAPLE_PWR_BASE->CR;
  /* Clear PDDS and LPDS bits */
  tmpreg &= CR_DS_MASK;
  /* Set LPDS bit according to PWR_Regulator value */
  tmpreg |= PWR_Regulator;
  /* Store the new value */
  MAPLE_PWR_BASE->CR = tmpreg;
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  MAPLE_SCB_BASE->SCR |= MAPLE_SCB_SCR_SLEEPDEEP;

//  RCC_BASE_MAPLE->BDCR = 1<<RTCEN
//  RCC_BASE_MAPLE->CSR=1<<LSION
  /* Select STOP mode entry --------------------------------------------------*/
  if(PWR_STOPEntry == PWR_STOPEntry_WFI)
  {
    /* Request Wait For Interrupt */
      __asm volatile ("wfi");
  }
  else
  {
    /* Request Wait For Event */
      __asm volatile ("wfe");
  }

  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  MAPLE_SCB_BASE->SCR &= (uint32_t)~((uint32_t)MAPLE_SCB_SCR_SLEEPDEEP);

  /**Note: To enter Stop mode,
   * all EXTI Line pending bits (in Pending register (EXTI_PR)),
   *  all peripheral interrupt pending bits,
   *  and RTC Alarm flag must be reset. Otherwise,
   *  the Stop mode entry procedure is ignored and
   *  program execution continues**/
}


/**
  * @brief  Enables or disables access to the RTC and backup registers.
  * @param  NewState: new state of the access to the RTC and backup registers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void pwr_bkp_access_cmd(uint8_t NewState)
{
  /* Check the parameters */
  *(__io uint32_t *) CR_DBP_BB = (uint32_t)NewState&0x01;
}

/**
  * @brief  Enables or disables the Power Voltage Detector(PVD).
  * @param  NewState: new state of the PVD.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void pwr_pvd_cmd(uint8_t NewState)
{
  /* Check the parameters */
  *(__io uint32_t *) CR_PVDE_BB = (uint32_t)NewState&0x01;
}

/**
  * @brief  Checks whether the specified PWR flag is set or not.
  * @param  PWR_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  *     @arg PWR_FLAG_PVDO: PVD Output
  * @retval The new state of PWR_FLAG (SET or RESET).
  */
uint8_t pwr_getflag_status(uint32_t PWR_FLAG)
{
  /* Check the parameters */
  if ((MAPLE_PWR_BASE->CSR & PWR_FLAG) != (uint32_t)0)
  {
    return  1;
  }
  /* Return the flag status */
  return 0;
}

/**
  * @brief  Clears the PWR's pending flags.
  * @param  PWR_FLAG: specifies the flag to clear.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  * @retval None
  */
void pwr_clrflag(uint32_t PWR_FLAG)
{
  /* Check the parameters */

  MAPLE_PWR_BASE->CR |=  PWR_FLAG << 2;
}


/**
  * @brief  Configures the voltage threshold detected by the Power Voltage Detector(PVD).
  * @param  PWR_PVDLevel: specifies the PVD detection level
  *   This parameter can be one of the following values:
  *     @arg PWR_PVDLevel_2V2: PVD detection level set to 2.2V
  *     @arg PWR_PVDLevel_2V3: PVD detection level set to 2.3V
  *     @arg PWR_PVDLevel_2V4: PVD detection level set to 2.4V
  *     @arg PWR_PVDLevel_2V5: PVD detection level set to 2.5V
  *     @arg PWR_PVDLevel_2V6: PVD detection level set to 2.6V
  *     @arg PWR_PVDLevel_2V7: PVD detection level set to 2.7V
  *     @arg PWR_PVDLevel_2V8: PVD detection level set to 2.8V
  *     @arg PWR_PVDLevel_2V9: PVD detection level set to 2.9V
  * @retval None
  */
void pwr_PVDlevelset(uint32_t PWR_PVDLevel)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */

  tmpreg = MAPLE_PWR_BASE->CR;
  /* Clear PLS[7:5] bits */
  tmpreg &= CR_PLS_MASK;
  /* Set PLS[7:5] bits according to PWR_PVDLevel value */
  tmpreg |= PWR_PVDLevel;
  /* Store the new value */
  MAPLE_PWR_BASE->CR = tmpreg;
}

/**
  * @brief  Enables or disables the WakeUp Pin functionality.
  * @param  NewState: new state of the WakeUp Pin functionality.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  *
  * his bit is set and cleared by software.
      0: WKUP pin is used for general purpose I/O. An event on the WKUP
     pin does not wakeup the device from Standby mode.
      1: WKUP pin is used for wakeup from Standby mode and forced in input pull down
     configuration (rising edge on WKUP pin wakes-up the system from Standby mode).
  */
void pwr_wakeup_pincmd(uint8_t NewState)
{
  /* Check the parameters */

  *(__io uint32_t *) CSR_EWUP_BB = (uint32_t)NewState & 0x01;
}

/**
 *
 */
void exti17_func(void){
  if(rtc_getit_status(RTC_IT_ALR) != 0)
    {
//      /* Check if the Wake-Up flag is set */
      if(pwr_getflag_status(PWR_FLAG_WU) != 0)
        {
          /* Clear Wake Up flag */
          pwr_clrflag(PWR_FLAG_WU);
        }
      //togglePin(PB12);
      rtc_setalarm(rtc_getcounter()+ 3);
      /* Wait until last write operation on RTC registers has finished */
      rtc_waitfor_lasttask();
      /* Clear RTC Alarm interrupt pending bit */
      rtc_clritpendbit(RTC_IT_ALR);
      /* Wait until last write operation on RTC registers has finished */
      rtc_waitfor_lasttask();
    }
}

/**
 *
 * @param clksrc
 */
void pwr_stopinit(uint8_t clksrc){
  //  /* Configure EXTI Line to generate an interrupt on falling edge */
    exti_attach_interrupt(EXTI17,EXTI_PA,exti17_func,EXTI_RISING);
  //   /* Configure RTC clock source and prescaler */
    rtc_config(clksrc);
  //   /* NVIC configuration */
    nvic_irq_set_priority(NVIC_RTCALARM,0);
}
/**
 *
 * @param tim
 */

void pwr_stopGo(uint32_t tim){

  rtc_clrflag(RTC_FLAG_SEC);
  while(rtc_getflagstatus(RTC_FLAG_SEC) == 0);
//  ///* Alarm in 3 second */
  rtc_setalarm(rtc_getcounter()+ tim);
//  ///* Wait until last write operation on RTC registers has finished */
  rtc_waitfor_lasttask();

///* Request to enter STOP mode with regulator in low power mode*/
  pwr_stopmode(0, PWR_STOPEntry_WFI);

}

