/**
 * @file stm32.c
 *
 * @date Mehr 12, 1395 AP
 * @author jupiter
 * @description
 */

#include <stdint.h>


uint32_t F_CPU=8000000l;
//uint32_t STM32_PCLK1   =                  F_CPU/2;
//uint32_t STM32_PCLK2       =              F_CPU;
//uint32_t STM32_DELAY_US_MULT     =   (F_CPU / 6000000L);
//uint32_t SYSTICK_RELOAD_VAL   =  (F_CPU/1000) - 1 ;/* takes a cycle to reload */
//uint32_t CYCLES_PER_MICROSECOND = F_CPU/1000000l;

 void set_CPUClock(uint32_t val){
  F_CPU=val;
}
 uint32_t get_PCLK1Clock(void)
{
  return F_CPU/2;
}

 uint32_t get_PCLK2Clock(void)
{
  return F_CPU;
}

 uint32_t get_delay(void)
{
  return (F_CPU / 6000000L);
}

 uint32_t get_systeckPreLoad(void)
{
  return (F_CPU/1000) - 1 ;/* takes a cycle to reload */
}

 uint32_t get_cycle(void)
{
  return F_CPU/1000000l;
}

 uint32_t get_CPUClock(){
   return F_CPU;
 }

