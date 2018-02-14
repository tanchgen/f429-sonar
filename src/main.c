//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f4xx.h"
#include "netconf.h"
#include "stm32f4x7_eth.h"

#include "my_time.h"
#include "tim.h"
#include "dac_adc.h"
#include "main.h"

extern tTims tims;

uint32_t count = 0;
uint16_t inter1 = 0;
uint16_t inter2 = 0;
uint32_t old = 0;

uint32_t mainModeCount = 0;

// Private functions ---------------------------------------------------------

int main( void ) {
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");
  SetSYSCLK_180();

  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  tInit();
  tim8Init();
  tim3Init();
  tim2Init();
  tim1Init();
  tim5Init();
  tim4Init();

  dacInit();
  adcInit();

  udpServerInit();

// Запуск Главного Таймера
  mainModeSet();
  // Для тестирования работы Главного Цикла
  if( tims.mainMode ){
    mainModeCount = 1300;
  }
  else{
    mainModeCount = 2000;
  }

  while (1) {
    // check if any packet received
    if (ETH_CheckFrameReceived()) {
      // process received ethernet packet
      LwIP_Pkt_Handle();
    }
    // handle periodic timers for LwIP
    LwIP_Periodic_Handle(myTick);

    if( mainModeCount == 1 ){
      if( tims.mainMode ){
        GPIOD->BSRRH |= GPIO_Pin_4;
        myDelay(1);
        GPIOD->BSRRL |= GPIO_Pin_4;
        mainModeCount = 1300;
      }
      else{
  //	    myDelay(2000);
  //  	  mainTimStop();
        mainTimStart();
        mainModeCount = 2000;
      }
    }
  }
}

// ----------------------------------------------------------------------------
