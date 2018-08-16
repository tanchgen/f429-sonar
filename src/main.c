//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "netconf.h"
#include "stm32f4x7_eth.h"

#include "my_time.h"
#include "tim.h"
#include "dac_adc.h"
#include "tcp_srv.h"
//#include "eth.h"
#include "main.h"

extern tTims tims;

uint32_t count = 0;
uint16_t inter1 = 0;
uint16_t inter2 = 0;
uint32_t old = 0;

uint32_t mainModeCount = 0;

// Private functions ---------------------------------------------------------
//struct pbuf * outBufInit( void );

int main( void ) {
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");
  SetSYSCLK_180();

  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);

  tInit();
  tim8Init();
  tim3Init();
  tim2Init();
  tim1Init();
  tim5Init();
  tim4Init();

  dacInit();
  adcInit();

  ethInit();
  udpInit();
  serverSetup();
  // Отправка широковещательного UDP-пакета: Я здесь - по адресу ..., слушаю порт ...
  bcstSend();

// Запуск Главного Таймера
//    myDelay(1500);
    timPrestart();
    mainModeSet();

 // Для тестирования работы Главного Цикла
  if( tims.mainMode == 0){
    mainModeCount = tims.T1Main;
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
      if( tims.mainMode == 0){
        // Ркжим работы - периодический автозапуск
        mainTimStart();
        mainModeCount = tims.T1Main;
      }
#if EXT_START
//      else{
      // Для теста: Запуск от внешнего пина PD4
//        GPIOD->BSRRH |= GPIO_Pin_4;
//        mDelay(1);
//        GPIOD->BSRRL |= GPIO_Pin_4;
//        mainModeCount = 1300;
//      }
#endif // EXT_START
    }
  }
}

// ----------------------------------------------------------------------------
