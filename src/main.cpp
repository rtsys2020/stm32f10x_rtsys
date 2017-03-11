////
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include <wirish.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include "task.h"
#include "minmea.h"
#include <SerialFlash.h>
#include <HardwareSerial.h>
#include <HardwareTimer.h>
#include "axp209.h"


extern void setup_clocks_8hsi(void) ;
void setup_clocks_64hse(void) ;
extern void setup_clocks_72hse();
/*
 * These addresses are where usercode starts when a bootloader is
 * present. If no bootloader is present, the user NVIC usually starts
 * at the Flash base address, 0x08000000.
 */
#if defined(BOOTLOADER_maple)
#define USER_ADDR_ROM 0x08005000
#else
#if defined(BOOTLOADER_robotis)
#define USER_ADDR_ROM 0x08003000
#else
#define USER_ADDR_ROM 0x08000000
#endif
#endif
#define USER_ADDR_RAM 0x20000C00






// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus
} // extern "C"
#endif


#include "cmsis_os.h"

typedef enum
{
  THREAD_1 = 0,
  THREAD_2,
  THREAD_3,
  THREAD_4
} Thread_TypeDef;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LoopThread1Handle, LoopThread2Handle,LoopThread3Handle,LoopThread4Handle;
/* Private function prototypes -----------------------------------------------*/


extern void GuiTask(void const *argument);
extern void RCSwitchTask(void const *argument);
extern void HMC8885Task(void const *argument);
extern void KeypadTask(void const *argument);
extern void PC_ConnectionTask(void const *argument);

AXP209_Charger axp209(PA9,PA10,5);

int pc_connection=0;

int main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");
  boardInit();
  USBserial.println("usb serial write data");

  Serial.begin(9600);

  //  pinMode(PB0,OUTPUT);
  //  pinMode(PB1,OUTPUT);

  systick_init(get_systeckPreLoad());
  //  for(int i=0;i<100;i++)
  //    {
  //      digitalWrite(PB0,0);
  //      digitalWrite(PB1,0);
  //      Serial.println("Hello!");
  ////      delay(1000);
  //      digitalWrite(PB0,1);
  //      digitalWrite(PB1,1);
  ////      delay(1000);
  //    }

  SPIClass2.begin();
  SPIClass2.setClockDivider(SPI_CLOCK_DIV2); //4MHz
  SPIClass2.setDataMode(SPI_MODE0);

  axp209.begin();
  uint8_t power_status = axp209.powerInputStatus();
  if(power_status & (1<<ACIN_OR_VBUS_SOURCE_BIT)){
      pc_connection=1;
  }

  uint8_t val = axp209.powerInputStatus();
  if(val&(1<<VBUS_PRESENCE_BIT))
    {
      USBserial.print("usb Connected:");
      USBserial.println(val,HEX);
    }

  if(pc_connection!=1){
      osThreadDef(THREAD_1, GuiTask, osPriorityNormal, 0, 1024);
      LoopThread1Handle = osThreadCreate(osThread(THREAD_1), NULL);
  }
  //  osThreadDef(THREAD_2, HMC8885Task, osPriorityNormal, 0, 2048);
  //  LoopThread2Handle = osThreadCreate(osThread(THREAD_2), NULL);

  if(pc_connection){
      osThreadDef(THREAD_3, PC_ConnectionTask, osPriorityNormal, 0, 2048);
      LoopThread3Handle = osThreadCreate(osThread(THREAD_3), NULL);
  }
  else{
      osThreadDef(THREAD_3, KeypadTask, osPriorityNormal, 0, 2048);
      LoopThread3Handle = osThreadCreate(osThread(THREAD_3), NULL);
  }
  //
  //
  //  /* Start scheduler */
  osKernelStart();


  for (;;){

      delay(10);
  }
  // Infinite loop, never return.
}


/**
 * @brief  Toggle LED2 thread
 * @param  thread not used
 * @retval None
 */




#pragma GCC diagnostic pop

//// ----------------------------------------------------------------------------
