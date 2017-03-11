/**
 * @file rtc_f1.c
 *
 * @date Mehr 5, 1395 AP
 * @author jupiter
 * @description
 */
#include "libmaple/rtc.h"


rtc_channel rtc_channels[]={
    { .handler = NULL, .arg = NULL },  // rtc_sec
    { .handler = NULL, .arg = NULL },  // rtc_ov
    { .handler = NULL, .arg = NULL },  // rtc_alarm
};

void rtc_attach_interrupt(
    voidFuncPtr handler,
    rtc_flag flag){
  /* Register the handler */

  rtc_channels[flag].handler = handler;
  rtc_channels[flag].arg = NULL;

}
void rtc_attach_callback(
    voidArgumentFuncPtr handler,
    void *arg,
    rtc_flag flag){
  rtc_channels[flag].handler = handler;
  rtc_channels[flag].arg = arg;

}

void rtc_detach_interrupt(rtc_flag flag){
  rtc_channels[flag].handler = NULL;
  rtc_channels[flag].arg = NULL;

}

static inline void dispatch_single_rtc(rtc_flag flag){
  voidArgumentFuncPtr handler = rtc_channels[flag].handler;
  if (!handler) {
      return;
  }
  handler(rtc_channels[flag].arg);
}



void rtc_init(){
  uint32_t tmpreg = 0;


  //  /* Select the regulator state in STOP mode ---------------------------------*/
  //  tmpreg = PWR_BASE_MAPLE->CR;
  //  //DBP bit in the Power control register (PWR_CR) has to be set
  //  bb_peri_set_bit(&tmpreg, RCC_BDCR_LSEBYP_BIT, 1);
  bkp_init();
  //  rcc_clk_enable(RCC_BKP);

  //  //   =
  //  tmpreg = RCC_BASE_MAPLE->BDCR;
  //  bb_peri_set_bit(&tmpreg, RTCEN_BIT, 1);
  //
  //  //lsi on
  //  RCC_BASE_MAPLE->CSR|=1;
  //  while(!rtc_is_lsi_ready())
  //    ;

}

int rtc_is_lsi_ready(void){
  return MAPLE_RCC_BASE->CSR&(0x00000002);
}

void rtc_entercfg(void)
{
  /* Set the CNF flag to enter in the Configuration Mode */

  RTC_BASE_MAPLE->CRL |= RTC_CRL_CNF;
}

/**
 * @brief  Exits from the RTC configuration mode.
 * @param  None
 * @retval None
 */
void rtc_exitcfg(void)
{
  /* Reset the CNF flag to exit from the Configuration Mode */
  RTC_BASE_MAPLE->CRL &= (uint16_t)~((uint16_t)RTC_CRL_CNF);
}

void rtc_setprescale(uint32_t PrescalerValue)
{
  /* Check the parameters */
  rtc_entercfg();
  /* Set RTC PRESCALER MSB word */
  RTC_BASE_MAPLE->PRLH = (PrescalerValue & PRLH_MSB_MASK) >> 16;
  /* Set RTC PRESCALER LSB word */
  RTC_BASE_MAPLE->PRLL = (PrescalerValue & RTC_LSB_MASK);
  rtc_exitcfg();
}

/**
 * @brief  Enables or disables the specified RTC interrupts.
 * @param  RTC_IT: specifies the RTC interrupts sources to be enabled or disabled.
 *   This parameter can be any combination of the following values:
 *     @arg RTC_IT_OW: Overflow interrupt
 *     @arg RTC_IT_ALR: Alarm interrupt
 *     @arg RTC_IT_SEC: Second interrupt
 * @param  NewState: new state of the specified RTC interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 * @retval None
 */
void rtc_itcfg(uint16_t RTC_IT, uint8_t NewState)
{
  /* Check the parameters */


  if (NewState != 0)
    {
      RTC_BASE_MAPLE->CRH |= RTC_IT;
    }
  else
    {
      RTC_BASE_MAPLE->CRH &= (uint16_t)~RTC_IT;
    }
}

void rtc_clrit_flag(uint16_t RTC_IT)
{
  /* Check the parameters */


  /* Clear the corresponding RTC pending bit */
  RTC_BASE_MAPLE->CRL &= (uint16_t)~RTC_IT;
}

/**
 * @brief  Sets the RTC alarm value.
 * @param  AlarmValue: RTC alarm new value.
 * @retval None
 */
void rtc_setalarm(uint32_t AlarmValue)
{
  rtc_entercfg();
  /* Set the ALARM MSB word */
  RTC_BASE_MAPLE->ALRH = AlarmValue >> 16;
  /* Set the ALARM LSB word */
  RTC_BASE_MAPLE->ALRL = (AlarmValue & RTC_LSB_MASK);
  rtc_exitcfg();
}


/**
 * @brief  Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
 *   are synchronized with RTC APB clock.
 * @note   This function must be called before any read operation after an APB reset
 *   or an APB clock stop.
 * @param  None
 * @retval None
 */
void rtc_waitfor_sync(void)
{
  /* Clear RSF flag */
  RTC_BASE_MAPLE->CRL &= (uint16_t)~RTC_FLAG_RSF;
  /* Loop until RSF flag is set */
  while ((RTC_BASE_MAPLE->CRL & RTC_FLAG_RSF) == (uint16_t)0)
    {
    }
}

/**
 * @brief  Checks whether the specified RTC flag is set or not.
 * @param  RTC_FLAG: specifies the flag to check.
 *   This parameter can be one the following values:
 *     @arg RTC_FLAG_RTOFF: RTC Operation OFF flag
 *     @arg RTC_FLAG_RSF: Registers Synchronized flag
 *     @arg RTC_FLAG_OW: Overflow flag
 *     @arg RTC_FLAG_ALR: Alarm flag
 *     @arg RTC_FLAG_SEC: Second flag
 * @retval The new state of RTC_FLAG (SET or RESET).
 */
uint8_t rtc_getflagstatus(uint16_t RTC_FLAG)
{
  if ((RTC_BASE_MAPLE->CRL & RTC_FLAG) != (uint16_t)0)
    {
      return 1;
    }
  return 0;
}

/**
 * @brief  Waits until last write operation on RTC registers has finished.
 * @note   This function must be called before any write to RTC registers.
 * @param  None
 * @retval None
 */
void rtc_waitfor_lasttask(void)
{
  /* Loop until RTOFF flag is set */
  while ((RTC_BASE_MAPLE->CRL & RTC_FLAG_RTOFF) == (uint16_t)0)
    {
    }
}


/**
 * @brief  Checks whether the specified RTC interrupt has occurred or not.
 * @param  RTC_IT: specifies the RTC interrupts sources to check.
 *   This parameter can be one of the following values:
 *     @arg RTC_IT_OW: Overflow interrupt
 *     @arg RTC_IT_ALR: Alarm interrupt
 *     @arg RTC_IT_SEC: Second interrupt
 * @retval The new state of the RTC_IT (SET or RESET).
 */
uint8_t rtc_getit_status(uint16_t RTC_IT)
{
  uint32_t bitstatus = 0;
  /* Check the parameters */


  bitstatus = (uint32_t)(RTC_BASE_MAPLE->CRL & RTC_IT);
  if (((RTC_BASE_MAPLE->CRH & RTC_IT) != (uint16_t)0) && (bitstatus != (uint16_t)0))
    {
      return 1;
    }
  return 0;
}


/**
 * @brief  Clears the RTC's interrupt pending bits.
 * @param  RTC_IT: specifies the interrupt pending bit to clear.
 *   This parameter can be any combination of the following values:
 *     @arg RTC_IT_OW: Overflow interrupt
 *     @arg RTC_IT_ALR: Alarm interrupt
 *     @arg RTC_IT_SEC: Second interrupt
 * @retval None
 */
void rtc_clritpendbit(uint16_t RTC_IT)
{
  /* Check the parameters */
  /* Clear the corresponding RTC pending bit */
  RTC_BASE_MAPLE->CRL &= (uint16_t)~RTC_IT;
}

uint32_t rtc_getcounter(void)
{
  uint16_t tmp = 0;
  tmp = RTC_BASE_MAPLE->CNTL;
  return (((uint32_t)RTC_BASE_MAPLE->CNTH << 16 ) | tmp) ;
}

/**
 * @brief  Sets the RTC counter value.
 * @param  CounterValue: RTC counter new value.
 * @retval None
 */
void rtc_setcounter(uint32_t CounterValue)
{
  RTC_EnterConfigMode();
  /* Set RTC COUNTER MSB word */
  RTC_BASE_MAPLE->CNTH = CounterValue >> 16;
  /* Set RTC COUNTER LSB word */
  RTC_BASE_MAPLE->CNTL = (CounterValue & RTC_LSB_MASK);
  RTC_ExitConfigMode();
}


/**
 * @brief  Gets the RTC divider value.
 * @param  None
 * @retval RTC Divider value.
 */
uint32_t rtc_getdiv(void)
{
  uint32_t tmp = 0x00;
  tmp = ((uint32_t)RTC_BASE_MAPLE->DIVH & (uint32_t)0x000F) << 16;
  tmp |= RTC_BASE_MAPLE->DIVL;
  return tmp;
}
/**
 * @brief  Clears the RTC's pending flags.
 * @param  RTC_FLAG: specifies the flag to clear.
 *   This parameter can be any combination of the following values:
 *     @arg RTC_FLAG_RSF: Registers Synchronized flag. This flag is cleared only after
 *                        an APB reset or an APB Clock stop.
 *     @arg RTC_FLAG_OW: Overflow flag
 *     @arg RTC_FLAG_ALR: Alarm flag
 *     @arg RTC_FLAG_SEC: Second flag
 * @retval None
 */
void rtc_clrflag(uint16_t RTC_FLAG)
{
  /* Check the parameters */

  /* Clear the corresponding RTC flag */
  RTC_BASE_MAPLE->CRL &= (uint16_t)~RTC_FLAG;
}


void RTC_IRQHandler(void)
{
  if (rtc_getit_status(RTC_IT_SEC) != 0)
    {
      dispatch_single_rtc(RTC_SEC_FLAG);
    }
  else   if (rtc_getit_status(RTC_IT_ALR) != 0)
    {
      dispatch_single_rtc(RTC_ALARM_FLAG);
    }
  else   if (rtc_getit_status(RTC_IT_OW) != 0)
    {
      dispatch_single_rtc(RTC_OW_FLAG);
    }
}

/**
 *
 * @param rtc_clk 0 lsi for rtc clock
 *                1 lse for rtc clock
 */
void rtc_config(uint8_t rtc_clk)
{
  //  /* RTC clock source configuration ------------------------------------------*/
  //  /* Allow access to BKP Domain */
  rtc_init();
  pwr_bkp_access_cmd(1);
  //
  //  /* Reset Backup Domain */
  bkp_reset_cmd(1);

  bkp_reset_cmd(0);

  if(rtc_clk){
      rcc_turn_on_clk(MAPLE_RCC_CLK_LSI);
      while(!rcc_is_clk_ready(MAPLE_RCC_CLK_LSI))
        ;
      //  /* Select the RTC Clock Source */
      MAPLE_RCC_BASE->BDCR |= MAPLE_RCC_RTCCLKSource_LSI;
  }
  else
    {
//      rcc_lseconfig(RCC_LSE_ON);
//      /* Wait till LSE is ready */
//      while(rcc_getflagstatus(RCC_FLAG_LSERDY) == 0)
//        ;

      rcc_turn_on_clk(MAPLE_RCC_CLK_LSE);
      while(!rcc_is_clk_ready(MAPLE_RCC_CLK_LSE))
        ;
      //  /* Select the RTC Clock Source */
      MAPLE_RCC_BASE->BDCR |= MAPLE_RCC_RTCCLKSource_LSE;
    }



  //

  //  /* Enable the RTC Clock */
  rcc_rtcclk_cmd(1);
  //  /* RTC configuration -------------------------------------------------------*/
  //  /* Wait for RTC APB registers synchronisation */
  rtc_waitfor_sync();
  //  /* Set the RTC time base to 1s */
  if(rtc_clk){
      rtc_setprescale(40000);
  }else{
      rtc_setprescale(32767);
  }
  //
  //  /* Wait until last write operation on RTC registers has finished */
  rtc_waitfor_lasttask();
  //  /* Enable the RTC Alarm interrupt */
  rtc_itcfg(RTC_IT_ALR,1);
  //  /* Wait until last write operation on RTC registers has finished */
  rtc_waitfor_lasttask();
}

/**
 *
 *  the RTC Alarm value stored in the Alarm register increased by one (RTC_ALR + 1).
 *  The write operation in the RTC Alarm and RTC Second flag must be synchronized
 *  by using one of the following sequences:
    • Use the RTC Alarm interrupt and inside the RTC interrupt routine,
the RTC Alarm and/or RTC Counter registers are updated.
    • Wait for SECF bit to be set in the RTC Control register.
Update the RTC Alarm and/or the RTC Counter register.
 */
