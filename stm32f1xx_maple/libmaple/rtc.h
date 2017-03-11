/**
 * @file rtc.h
 *
 * @date Mehr 5, 1395 AP
 * @author jupiter
 * @description
 */

#ifndef LIBMAPLE_RTC_H_
#define LIBMAPLE_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <libmaple/libmaple.h>
  /* Roger clark. Replaced with line below #include <series/pwr.h>*/
#include "system/pwr.h"
#include <libmaple/rcc.h>
#include <libmaple/libmaple.h>
#include <libmaple/bitband.h>

#include "libmaple/rcc_private.h"
#include <libmaple/rcc.h>
#include <libmaple/scb.h>


  typedef struct rtc_reg_map
  {
    __io uint16_t CRH;
    uint16_t  RESERVED0;
    __io uint16_t CRL;
    uint16_t  RESERVED1;
    __io uint16_t PRLH;
    uint16_t  RESERVED2;
    __io uint16_t PRLL;
    uint16_t  RESERVED3;
    __io uint16_t DIVH;
    uint16_t  RESERVED4;
    __io uint16_t DIVL;
    uint16_t  RESERVED5;
    __io uint16_t CNTH;
    uint16_t  RESERVED6;
    __io uint16_t CNTL;
    uint16_t  RESERVED7;
    __io uint16_t ALRH;
    uint16_t  RESERVED8;
    __io uint16_t ALRL;
    uint16_t  RESERVED9;
  } rtc_reg_map;


#define RTC_BASE_MAPLE                        ((struct rtc_reg_map*)0x40002800)

#define  RTC_CRL_SECF                        ((uint8_t)0x01)               /*!< Second Flag */
#define  RTC_CRL_ALRF                        ((uint8_t)0x02)               /*!< Alarm Flag */
#define  RTC_CRL_OWF                         ((uint8_t)0x04)               /*!< OverfloW Flag */
#define  RTC_CRL_RSF                         ((uint8_t)0x08)               /*!< Registers Synchronized Flag */
#define  RTC_CRL_CNF                         ((uint8_t)0x10)               /*!< Configuration Flag */
#define  RTC_CRL_RTOFF                       ((uint8_t)0x20)               /*!< RTC operation OFF */


#define RTC_IT_OW            ((uint16_t)0x0004)  /*!< Overflow interrupt */
#define RTC_IT_ALR           ((uint16_t)0x0002)  /*!< Alarm interrupt */
#define RTC_IT_SEC           ((uint16_t)0x0001)  /*!< Second interrupt */

#define RTC_FLAG_RTOFF       ((uint16_t)0x0020)  /*!< RTC Operation OFF flag */
#define RTC_FLAG_RSF         ((uint16_t)0x0008)  /*!< Registers Synchronized flag */
#define RTC_FLAG_OW          ((uint16_t)0x0004)  /*!< Overflow flag */
#define RTC_FLAG_ALR         ((uint16_t)0x0002)  /*!< Alarm flag */
#define RTC_FLAG_SEC         ((uint16_t)0x0001)  /*!< Second flag */

#define RTC_LSB_MASK     ((uint32_t)0x0000FFFF)  /*!< RTC LSB Mask */
#define PRLH_MSB_MASK    ((uint32_t)0x000F0000)  /*!< RTC Prescaler MSB Mask */

#define RTCEN_BIT 15
#define LSEBYP_BIT 2
#define LSERDY_BIT 1
#define LSEON_BIT 0

#define RTCSEL_MASK (0x11<<8)

  typedef enum{
    RTC_SRC_NONE,
    RTC_SRC_LSE,
    RTC_SRC_LSI,
    RTC_SRC_HSE
  }rtc_src;

  typedef enum{
    RTC_SEC_FLAG,
    RTC_OW_FLAG,
    RTC_ALARM_FLAG
  }rtc_flag;


  typedef struct  {
    void (*handler)(void *);
    void *arg;
  } rtc_channel;



  void rtc_attach_interrupt(
      voidFuncPtr handler,
      rtc_flag flag);
  void rtc_attach_callback(
      voidArgumentFuncPtr handler,
      void *arg,
      rtc_flag flag);
  void rtc_detach_interrupt(rtc_flag flag);


  void rtc_clritpendbit(uint16_t RTC_IT);
  uint8_t rtc_getit_status(uint16_t RTC_IT);
  void rtc_waitfor_lasttask(void);
  void rtc_waitfor_sync(void);
  void rtc_setalarm(uint32_t AlarmValue);
  void rtc_clrit_flag(uint16_t RTC_IT);
  void rtc_itcfg(uint16_t RTC_IT, uint8_t NewState);
  void rtc_setprescale(uint32_t PrescalerValue);
  void rtc_exitcfg(void);
  void rtc_entercfg(void);
  uint32_t rtc_getcounter(void);
  void rtc_clrflag(uint16_t RTC_FLAG);
  uint8_t rtc_getflagstatus(uint16_t RTC_FLAG);
  void rtc_config(uint8_t rtc_clk);
#ifdef __cplusplus
}
#endif

#endif /* LIBMAPLE_RTC_H_ */
