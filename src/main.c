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
#include "eth.h"
#include "main.h"

extern tTims tims;

uint32_t count = 0;
uint16_t inter1 = 0;
uint16_t inter2 = 0;
uint32_t old = 0;

uint32_t mainModeCount = 0;

// Private functions ---------------------------------------------------------
//struct pbuf * outBufInit( void );
void mco2EthConfig( void );

int main( void ) {
  SetSYSCLK_180();
//  mco2EthConfig();

  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

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
        mDelay(1);
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

void mco2EthConfig( void ){
  // MCO2 source - PLLI2S
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2PRE);// | RCC_MCO2Div_4;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2) | RCC_MCO2Source_PLLI2SCLK;
//  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2) | RCC_MCO2Source_HSE;

  // Update SystemCoreClock global variable value
  SystemCoreClock = 180000000;
  RCC_GetClocksFreq( &RCC_Clocks );
  apb1TimClock = (RCC->CFGR & RCC_CFGR_PPRE1)? RCC_Clocks.PCLK1_Frequency * 2: RCC_Clocks.PCLK1_Frequency;
  apb2TimClock = (RCC->CFGR & RCC_CFGR_PPRE2)? RCC_Clocks.PCLK2_Frequency * 2: RCC_Clocks.PCLK2_Frequency;
}
// ----------------------------------------------------------------------------
