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
 * &file   wirish/boards/maple_mini/board.cpp
 * &author Marti Bolivar <mbolivar&leaflabs.com>
 * &brief  Maple Mini board file.
 */

#include <board.h>
#include "system/stm32.h"
#include <libmaple/gpio.h>
#include <libmaple/timer.h>

/* Roger Clark. Added next to includes for changes to Serial */
#include <libmaple/usart.h>
#include <HardwareSerial.h>

#include <wirish_debug.h>
#include <wirish_types.h>

#include <board.h>
#include <libmaple/gpio.h>
#include <libmaple/libmaple_types.h>
#include <libmaple/flash.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>
//#include <libmaple/systick.h>
#include <libmaple/rcc.h>
#include <libmaple/pwr.h>
#include <libmaple/usart.h>
#include "wirish.h"
#include "HardwareSerial.h"
#include "libmaple/rtc.h"






__weak adc_prescaler w_adc_pre = MAPLE_ADC_PRE_PCLK2_DIV_6;
__weak adc_smp_rate w_adc_smp = MAPLE_ADC_SMPR_55_5;

__weak void board_setup_clock_prescalers(void) {
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);
  rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_2);
  rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);
  rcc_clk_disable(RCC_USB);
#if F_CPU == 72000000
  rcc_set_prescaler(RCC_PRESCALER_USB, RCC_USB_SYSCLK_DIV_1_5);
#elif F_CPU == 48000000
  rcc_set_prescaler(RCC_PRESCALER_USB, RCC_USB_SYSCLK_DIV_1_5);
#endif
}
/**
 *
 */
void setup_clocks_8hsi(void) {
  // Turn on HSI. We'll switch to and run off of this while we're
  // setting up the main PLL.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSI);

  // Turn off and reset the clock subsystems we'll be using, as well
  // as the clock security subsystem (CSS). Note that resetting CFGR
  // to its default value of 0 implies a switch to HSI for SYSCLK.
  MAPLE_RCC_BASE->CFGR = 0x00000000;
  rcc_disable_css();
  rcc_turn_off_clk(MAPLE_RCC_CLK_PLL);
  rcc_turn_off_clk(MAPLE_RCC_CLK_HSE);
  // wirish::priv::board_reset_pll();
  // Clear clock readiness interrupt flags and turn off clock
  // readiness interrupts.
  MAPLE_RCC_BASE->CIR = 0x00000000;
  //    // Enable HSE, and wait until it's ready.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSI);
  while (!rcc_is_clk_ready(MAPLE_RCC_CLK_HSI))
    ;
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);
  rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_1);
  rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);

  set_CPUClock(8000000l);

}

void setup_clocks_64hse(void) {
  stm32f1_rcc_pll_data pll_data64 = {MAPLE_RCC_PLLMUL_16};
  rcc_pll_cfg w_pll_cfg = {MAPLE_RCC_PLLSRC_HSI_DIV_2, &pll_data64};
  // Turn on HSI. We'll switch to and run off of this while we're
  // setting up the main PLL.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSI);

  // Turn off and reset the clock subsystems we'll be using, as well
  // as the clock security subsystem (CSS). Note that resetting CFGR
  // to its default value of 0 implies a switch to HSI for SYSCLK.
  //  RCC_BASE_MAPLE->CFGR = 0x00000000;
  rcc_disable_css();
  rcc_turn_off_clk(MAPLE_RCC_CLK_PLL);
  rcc_turn_off_clk(MAPLE_RCC_CLK_HSE);
  // wirish::priv::board_reset_pll();
  // Clear clock readiness interrupt flags and turn off clock
  // readiness interrupts.
  MAPLE_RCC_BASE->CIR = 0x00000000;

  //    // Enable HSE, and wait until it's ready.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSI);
  while (!rcc_is_clk_ready(MAPLE_RCC_CLK_HSI))
    ;

  // Configure AHBx, APBx, etc. prescalers and the main PLL.
  board_setup_clock_prescalers();

  rcc_configure_pll(&w_pll_cfg);

  //    // Enable the PLL, and wait until it's ready.
  rcc_turn_on_clk(MAPLE_RCC_CLK_PLL);
  while(!rcc_is_clk_ready(MAPLE_RCC_CLK_PLL))
    ;
  //
  //    // Finally, switch to the now-ready PLL as the main clock source.
  rcc_switch_sysclk(MAPLE_RCC_CLKSRC_PLL);

  set_CPUClock(64000000l);

}


/**
 *
 */
void setup_clocks_72hse(void) {
  stm32f1_rcc_pll_data pll_data64 = {MAPLE_RCC_PLLMUL_9};
  rcc_pll_cfg w_pll_cfg = {MAPLE_RCC_PLLSRC_HSE, &pll_data64};
  // Turn on HSI. We'll switch to and run off of this while we're
  // setting up the main PLL.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSI);

  // Turn off and reset the clock subsystems we'll be using, as well
  // as the clock security subsystem (CSS). Note that resetting CFGR
  // to its default value of 0 implies a switch to HSI for SYSCLK.
  //  RCC_BASE_MAPLE->CFGR = 0x00000000;
  rcc_disable_css();
  rcc_turn_off_clk(MAPLE_RCC_CLK_PLL);
  rcc_turn_off_clk(MAPLE_RCC_CLK_HSE);
  // wirish::priv::board_reset_pll();
  // Clear clock readiness interrupt flags and turn off clock
  // readiness interrupts.
  MAPLE_RCC_BASE->CIR = 0x00000000;


  //    // Enable HSE, and wait until it's ready.
  rcc_turn_on_clk(MAPLE_RCC_CLK_HSE);
  while (!rcc_is_clk_ready(MAPLE_RCC_CLK_HSE))
    ;


  // Configure AHBx, APBx, etc. prescalers and the main PLL.
  board_setup_clock_prescalers();
  rcc_configure_pll(&w_pll_cfg);

  //    // Enable the PLL, and wait until it's ready.
  rcc_turn_on_clk(MAPLE_RCC_CLK_PLL);
  while(!rcc_is_clk_ready(MAPLE_RCC_CLK_PLL))
    ;
  //
  //    // Finally, switch to the now-ready PLL as the main clock source.
  rcc_switch_sysclk(MAPLE_RCC_CLKSRC_PLL);
  //  F_CPU=72000000l;
  set_CPUClock(72000000l);
  //  STM32_PCLK1   =                  F_CPU/2;
  //  STM32_PCLK2       =              F_CPU;
  //  STM32_DELAY_US_MULT     =   (F_CPU / 6000000L);
  //  SYSTICK_RELOAD_VAL   =  (F_CPU/1000) - 1 ;/* takes a cycle to reload */
  //  CYCLES_PER_MICROSECOND = F_CPU/1000000l;
}

void setup_nvic(void) {

  nvic_init((uint32)0x8000000, 0);

  /* Roger Clark. We now control nvic vector table in boards.txt using the build.vect paramater
#ifdef VECT_TAB_FLASH
    nvic_init(USER_ADDR_ROM, 0);
#elif defined VECT_TAB_RAM
    nvic_init(USER_ADDR_RAM, 0);
#elif defined VECT_TAB_BASE
    nvic_init((uint32)0x08000000, 0);
#elif defined VECT_TAB_ADDR
    // A numerically supplied value
    nvic_init((uint32)VECT_TAB_ADDR, 0);
#else
    // Use the __text_start__ value from the linker script; this
    // should be the start of the vector table.
    nvic_init((uint32)&__text_start__, 0);
#endif

   */
}

void setup_flash(void) {
  // Turn on as many Flash "go faster" features as
  // possible. flash_enable_features() just ignores any flags it
  // can't support.
  flash_enable_features(MAPLE_FLASHPREFETCH | MAPLE_FLASHICACHE | MAPLE_FLASHDCACHE);
  // Configure the wait states, assuming we're operating at "close
  // enough" to 3.3V.


  //  zero wait state, if 0 < SYSCLK ≤ 24 MHz
  //  flash_set_latency(FLASH_WAIT_STATE_0);
  //  one wait state, if 24 MHz < SYSCLK ≤ 48 MHz
  //  flash_set_latency(FLASH_WAIT_STATE_1);
  //  two wait states, if 48 MHz < SYSCLK ≤ 72 MHz
  flash_set_latency(MAPLE_FLASH_WAIT_STATE_2);
}



__weak void board_setup_usb(void) {
#ifdef SERIAL_USB

#ifdef GENERIC_BOOTLOADER
  //Reset the USB interface on generic boards - developed by Victor PV
  gpio_set_mode(PIN_MAP[PA12].gpio_device, PIN_MAP[PA12].gpio_bit, MAPLE_GPIO_OUTPUT_PP);
  gpio_write_bit(PIN_MAP[PA12].gpio_device, PIN_MAP[PA12].gpio_bit,0);

  for(volatile unsigned int i=0;i<512;i++);// Only small delay seems to be needed, and USB pins will get configured in Serial.begin
  gpio_set_mode(PIN_MAP[PA12].gpio_device, PIN_MAP[PA12].gpio_bit, MAPLE_GPIO_INPUT_FLOATING);
#endif
  USBserial.begin();// Roger Clark. Changed SerialUSB to Serial for Arduino sketch compatibility
#endif
}

void adc_default_config(const adc_dev *dev) {
  adc_enable_single_swstart(dev);
  adc_set_sample_rate(dev, MAPLE_ADC_SMPR_55_5);
}

void setup_adcs(void) {
  adc_set_prescaler(MAPLE_ADC_PRE_PCLK2_DIV_6);
  adc_foreach(adc_default_config);
}

void timer_default_config(timer_dev *dev) {
  timer_adv_reg_map *regs = (dev->regs).adv;
  const uint16 full_overflow = 0xFFFF;
  const uint16 half_duty = 0x8FFF;

  timer_init(dev);
  timer_pause(dev);

  regs->CR1 = TIMER_CR1_ARPE;
  regs->PSC = 1;
  regs->SR = 0;
  regs->DIER = 0;
  regs->EGR = TIMER_EGR_UG;
  switch (dev->type) {
    case TIMER_ADVANCED:
      regs->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF;
      // fall-through
    case TIMER_GENERAL:
      timer_set_reload(dev, full_overflow);
      for (uint8 channel = 1; channel <= 4; channel++) {
          if (timer_has_cc_channel(dev, channel)) {
              timer_set_compare(dev, channel, half_duty);
              timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1,
                                TIMER_OC_PE);
          }
      }
      // fall-through
    case TIMER_BASIC:
      break;
  }

  timer_generate_update(dev);
  timer_resume(dev);
}

void setup_timers(void) {
  timer_foreach(timer_default_config);
}



// Note. See the enum of pin names in board.h

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {


    {&gpioa, &timer2, &adc1,  0, 1,    0}, /* PA0 */
    {&gpioa, &timer2, &adc1,  1, 2,    1}, /* PA1 */
    {&gpioa, &timer2, &adc1,  2, 3,    2}, /* PA2 */	
    {&gpioa, &timer2, &adc1,  3, 4,    3}, /* PA3 */
    {&gpioa,   NULL, &adc1,  4, 0,    4}, /* PA4 */
    {&gpioa,   NULL, &adc1,  5, 0,    5}, /* PA5 */
    {&gpioa, &timer3, &adc1,  6, 1,    6}, /* PA6 */
    {&gpioa, &timer3, &adc1,  7, 2,    7}, /* PA7 */
    {&gpioa, &timer1, NULL,  8, 1, ADCx}, /* PA8 */	
    {&gpioa, &timer1, NULL,  9, 2, ADCx}, /* PA9 */	
    {&gpioa, &timer1, NULL, 10, 3, ADCx}, /* PA10 */
    {&gpioa, &timer1, NULL, 11, 4, ADCx}, /* PA11 */
    {&gpioa,   NULL, NULL, 12, 0, ADCx}, /* PA12 */	
    {&gpioa,   NULL, NULL, 13, 0, ADCx}, /* PA13 */	
    {&gpioa,   NULL, NULL, 14, 0, ADCx}, /* PA14 */
    {&gpioa,   NULL, NULL, 15, 0, ADCx}, /* PA15 */

    {&gpiob, &timer3, &adc1,  0, 3,    8}, /* PB0 */	
    {&gpiob, &timer3, &adc1,  1, 4,    9}, /* PB1 */
    {&gpiob,   NULL, NULL,  2, 0, ADCx}, /* PB2 */
    {&gpiob,   NULL, NULL,  3, 0, ADCx}, /* PB3 */
    {&gpiob,   NULL, NULL,  4, 0, ADCx}, /* PB4 */
    {&gpiob,   NULL, NULL,  5, 0, ADCx}, /* PB5 */
    {&gpiob, &timer4, NULL,  6, 1, ADCx}, /* PB6 */
    {&gpiob, &timer4, NULL,  7, 2, ADCx}, /* PB7 */	
    {&gpiob, &timer4, NULL,  8, 3, ADCx}, /* PB8 */	
    {&gpiob, &timer4, NULL,  9, 4, ADCx}, /* PB9 */	
    {&gpiob,   NULL, NULL, 10, 0, ADCx}, /* PB10 */	
    {&gpiob,   NULL, NULL, 11, 0, ADCx}, /* PB11 */
    {&gpiob,   NULL, NULL, 12, 0, ADCx}, /* PB12 */
    {&gpiob,   NULL, NULL, 13, 0, ADCx}, /* PB13 */
    {&gpiob,   NULL, NULL, 14, 0, ADCx}, /* PB14 */
    {&gpiob,   NULL, NULL, 15, 0, ADCx}, /* PB15 */

    {&gpioc,   NULL, NULL, 13, 0, ADCx}, /* PC13 */	
    {&gpioc,   NULL, NULL, 14, 0, ADCx}, /* PC14 */
    {&gpioc,   NULL, NULL, 15, 0, ADCx}, /* PC15 */



};

extern const uint8 boardPWMPins[] __FLASH__ = {
    PB0, PA7, PA6, PA3, PA2, PA1, PA0, PB7, PB6, PA10, PA9, PA8
};

extern const uint8 boardADCPins[] __FLASH__ = {
    PB0, PA7, PA6 , PA5 , PA4 , PA3 , PA2 , PA1 , PA0 
};

// Note. These defines are not really used by generic boards. They are for  Maple Serial USB
#define USB_DP PA12
#define USB_DM PA11

// NOte. These definitions are not really used for generic boards, they only relate to boards modified to behave like Maple boards
extern const uint8 boardUsedPins[] __FLASH__ = {
    USB_DP, USB_DM
};


/* 
 * Roger Clark
 * 
 * 2015/05/28
 *
 * Moved definitions for Hardware Serial devices from HardwareSerial.cpp so that each board can define which Arduino "Serial" instance
 * Maps to which hardware serial port on the microprocessor
 */


HardwareSerial Serial(USART1_MAPLE,BOARD_USART1_TX_PIN,BOARD_USART1_RX_PIN);
//HardwareSerial Serial1(USART2_MAPLE,BOARD_USART2_TX_PIN,BOARD_USART2_RX_PIN);
//HardwareSerial Serial2(USART3_MAPLE,BOARD_USART3_TX_PIN,BOARD_USART3_RX_PIN);



/* Since we want the Serial Wire/JTAG pins as GPIOs, disable both SW
 * and JTAG debug support, unless configured otherwise. */
void boardInit(void) {
#ifndef CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG
  //  disableDebugPorts();
#endif
  setup_flash();
  //  setup_clocks_8hsi();
//  setup_clocks_64hse() ;
  setup_clocks_72hse();
  setup_nvic();
  gpio_init_all();

  setup_adcs();
  setup_timers();
  afio_init();
  afio_cfg_debug_ports(MAPLE_AFIO_DEBUG_SW_ONLY);
  board_setup_usb();


  //  pinMode(PB2,OUTPUT);
  //  digitalWrite(PB2,1);
//  Serial.begin(9600);

}

