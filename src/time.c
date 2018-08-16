/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 * unix_time.c
 *      Author: Jet <g.tanchin@yandex.ru>
 *  Created on: 08 апр. 2016 г.
 */

#include "stm32f4xx.h"
#include "my_time.h"
#include "stm32f4xx_it.h"

volatile timer_ticks_t timer_delayCount;
volatile time_t uxTime;
volatile uint32_t myTick;
volatile uint32_t usDelFlag;

tDate sysDate;
tTime sysTime;

RCC_ClocksTypeDef RCC_Clocks;
uint32_t apb1TimClock;
uint32_t apb2TimClock;

uint32_t toReadCount;
uint8_t  secondFlag = RESET;


#if 1   // HSI - System Clock Source
void SetSYSCLK_180(void) {

  RCC->APB1ENR |= RCC_APB1Periph_PWR;

  FLASH->ACR |= 0x5;

  PWR->CR |= PWR_CR_VOS;

  PWR->CR |= PWR_CR_ODEN;
  while( (PWR->CSR & PWR_CSR_ODRDY) == 0 )
  {}

  PWR->CR |= PWR_CR_ODSWEN;
  while( (PWR->CSR & PWR_CSR_ODSWRDY) == 0 )
  {}

  RCC->CR |= (0x10 << 3);  // RCC_CR_HSITRIM = 0x10
//  RCC->CR |= RCC_CR_HSION;

   /* Wait till HSI is ready */
//  while( (RCC->CR & RCC_CR_HSIRDY) == 0)
//  {}

  RCC->CR |= RCC_CR_HSEON;
  // Wait till HSE is ready
  while( (RCC->CR & RCC_CR_HSERDY) == 0)
  {}

  RCC->CR &= ~RCC_CR_PLLON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLRDY) != 0)
  {}

  /* Configure PLL clock to have:
  PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N = 360 MHz
  SYSCLK = PLL_VCO / PLL_P = 180 MHz
  USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ = ~51 MHz (USB is not used)
  */
  //PLLM = 50, PLLN = 360, PLLP = 2, PLLQ = 7
  RCC->PLLCFGR = 50 | (360L << 6) | (((2 >> 1) -1) << 16) | 7 << 24 | (RCC_PLLSource_HSE);

  RCC->CR |= RCC_CR_PLLON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLRDY) == 0)
  {}

  // HCLK = SYSCLK/1
  RCC->CFGR &= ~RCC_CFGR_HPRE;
  // PCLK1 = HCLK/4
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4;
  // PCLK2 = HCLK/4\2
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV2;

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  while( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL )
  {}

  // MCO2 source - HSE
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO2PRE | RCC_CFGR_MCO2)) | RCC_MCO2Source_HSE;

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

  // Update SystemCoreClock global variable value
  SystemCoreClock = 180000000;
  RCC_GetClocksFreq( &RCC_Clocks );
  apb1TimClock = (RCC->CFGR & RCC_CFGR_PPRE1)? RCC_Clocks.PCLK1_Frequency * 2: RCC_Clocks.PCLK1_Frequency;
  apb2TimClock = (RCC->CFGR & RCC_CFGR_PPRE2)? RCC_Clocks.PCLK2_Frequency * 2: RCC_Clocks.PCLK2_Frequency;
}
#else
/**
* @brief  Configures System clock to 180 MHz.
* @param  None
* @retval None
*/
void SetSYSCLK_180(void) {
  // Select HSE as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

  // Wait till HSE is used as system clock source
  while (RCC_GetSYSCLKSource() != 0x04)
  {}

  // Disable PLL
  RCC_PLLCmd(DISABLE);

  /* Configure PLL clock to have:
  PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N = 360 MHz
  SYSCLK = PLL_VCO / PLL_P = 180 MHz
  USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ = ~51 MHz (USB is not used)
  */
  RCC_PLLConfig(RCC_PLLSource_HSE,
                8, //PLLM
                360, //PLLN
                2, //PLLP
                7 //PLLQ
                  );

  // Enable PLL
  RCC_PLLCmd(ENABLE);

  // Wait till PLL is ready
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}

  // Enable the Over-drive to extend the clock frequency to 180 Mhz
  PWR_OverDriveCmd(ENABLE);

  // Wait till Over-drive is ready
  while (PWR_GetFlagStatus(PWR_FLAG_ODRDY) == RESET)
  {}
  // Enable the Over-drive mode switching to extend the clock frequency to 180 Mhz
  PWR_OverDriveSWCmd(ENABLE);

  // Wait till Over-drive mode switching is ready
  while (PWR_GetFlagStatus(PWR_FLAG_ODSWRDY) == RESET)
  {}

  // Select PLL as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  // Wait till PLL is used as system clock source
  while (RCC_GetSYSCLKSource() != 0x08)
  {}

  // Update SystemCoreClock global variable value
  SystemCoreClock = 180000000;

  RCC_GetClocksFreq( &RCC_Clocks );
  apb1TimClock = (RCC->CFGR & RCC_CFGR_PPRE1)? RCC_Clocks.PCLK1_Frequency * 2: RCC_Clocks.PCLK1_Frequency;
  apb2TimClock = (RCC->CFGR & RCC_CFGR_PPRE2)? RCC_Clocks.PCLK2_Frequency * 2: RCC_Clocks.PCLK2_Frequency;

}

void SetSYSCLK_168(void) {
  /* Select HSE as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

  /* Wait till HSE is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x04)
  {}

  /* Disable PLL */
  RCC_PLLCmd(DISABLE);

  /* Disable Over-drive mode */
  PWR_OverDriveCmd(DISABLE);
  PWR_OverDriveSWCmd(DISABLE);

  /* Wait till ODSWRDY is reset */
  while (PWR_GetFlagStatus(PWR_FLAG_ODSWRDY) != RESET)
  {}

  /* Configure PLL clock to have:
  PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N = 336 MHz
  SYSCLK = PLL_VCO / PLL_P = 168 MHz
  USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ = 48 MHz
  */
  RCC_PLLConfig(RCC_PLLSource_HSE,
                8, //PLLM
                336, //PLLN
                2, //PLLP
                7 //PLLQ
                  );

  /* Enable PLL */
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08)
  {}

  /* Update SystemCoreClock global variable value */

  RCC_GetClocksFreq( &RCC_Clocks );
  SystemCoreClock = RCC_Clocks.SYSCLK_Frequency;
}
#endif

/*
// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void timeInit( void ) {
	RTC_InitTypeDef rtcInitStruct;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;

  RTC_StructInit( &rtcInitStruct );
  RTC_Init( &rtcInitStruct );
  //##-1- Configure the Date #################################################
  // Set Date: Wednesday June 1st 2016
  sdatestructure.RTC_Year = 16;
  sdatestructure.RTC_Month = RTC_Month_June;
  sdatestructure.RTC_Date = 1;
  sdatestructure.RTC_WeekDay = RTC_Weekday_Wednesday;

  if(RTC_SetDate( RTC_Format_BIN ,&sdatestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

  stimestructure.RTC_Hours = 0;
  stimestructure.RTC_Minutes = 0;
  stimestructure.RTC_Seconds = 0;

  if(RTC_SetTime( rtcInitStruct.RTC_HourFormat ,&stimestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

}
*/

// Получение системного мремени
uint32_t getTick( void ) {
	// Возвращает количество тиков
	return myTick;
}

uint32_t sys_now( void ){
	return myTick;
}
#define _TBIAS_DAYS		((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		0
#define MONTAB(year)		((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

time_t xTm2Utime( tDate *mdate, tTime *mtime ){
	/* convert time structure to scalar time */
int32_t		days;
int32_t		secs;
int32_t		mon, year;

	/* Calculate number of days. */
	mon = mdate->Month - 1;
	year = mdate->Year - _TBIAS_YEAR;
	days  = Daysto32(year, mon) - 1;
	days += 365 * year;
	days += mdate->Date;
	days -= _TBIAS_DAYS;

	/* Calculate number of seconds. */
	secs  = 3600 * mtime->Hours;
	secs += 60 * mtime->Minutes;
	secs += mtime->Seconds;

	secs += (days * (time_t)86400);

	return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( tDate * mdate, tTime *mtime, time_t secsarg){
	uint32_t		secs;
	int32_t		days;
	int32_t		mon;
	int32_t		year;
	int32_t		i;
	const int16_t *	pm;

	#ifdef	_XT_SIGNED
	if (secsarg >= 0) {
			secs = (uint32_t)secsarg;
			days = _TBIAS_DAYS;
		} else {
			secs = (uint32_t)secsarg + _TBIAS_SECS;
			days = 0;
		}
	#else
		secs = secsarg;
		days = _TBIAS_DAYS;
	#endif

		/* days, hour, min, sec */
	days += secs / 86400;
	secs = secs % 86400;
	mtime->Hours = secs / 3600;
	secs %= 3600;
	mtime->Minutes = secs / 60;
	mtime->Seconds = secs % 60;

	mdate->WeekDay = (days + 1) % 7;

	/* determine year */
	for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
	days -= i;
	mdate->Year = year + _TBIAS_YEAR;

		/* determine month */
	pm = MONTAB(year);
	for (mon = 12; days < pm[--mon]; );
	mdate->Month = mon + 1;
	mdate->Date = days - pm[mon] + 1;
}

//void timersHandler( void ) {
//
//	// Decrement to zero the counter used by the delay routine.
//  if (timer_delayCount != 0u) {
//    --timer_delayCount;
//  }
//
///*	// Таймаут для логгирования температуры
//	if ( toLogCount > 1) {
//		toLogCount--;
//	}
//*/
//	// Таймаут для считывания температуры
//	if ( mainModeCount > 1) {
//		mainModeCount--;
//	}
//
//	// Секундный таймер
//	if ( !(myTick % 1000) ) {
//		secondFlag = SET;
//		uxTime++;
//		xUtime2Tm( &sysDate, &sysTime, uxTime );
//	}
//
//
//}

void timersProcess( void ) {
/*
	// Таймаут для логгирования температуры
	if ( toLogCount == 1 ) {
		toLogCount = toLogTout+1;
		toLogWrite();
	}
*/
	// Флаг "Прошла еще  одна секунда"
	if ( secondFlag ) {
		secondFlag = RESET;
		if ( sysTime.Seconds == 0 ){
			// Каждую минуту отправляем текущее  положение задвижки
			// canSendMsg( VALVE_DEG, valve.curDeg );
		}
	}
}

// Инициализация таймера микросекундных задержек
void debounceInit( void ) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM_DeInit(DEBOUNCE_TIM);
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_CEN;
	// Выставляем счетчик на 0,5 мкс
	DEBOUNCE_TIM->PSC = (RCC_Clocks.PCLK2_Frequency/1000) - 1;			// Считаем миллисекунды
	DEBOUNCE_TIM->ARR = 49;								// До 50-и миллисекунд
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_CKD;
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_DIR;		// Считаем на возрастание
	DEBOUNCE_TIM->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ( DEBOUNCE_NVIC_IRQCHANNEL );
	NVIC_SetPriority( DEBOUNCE_NVIC_IRQCHANNEL, 0xF );
}

// Задержка в мс
void mDelay( uint32_t del ){
	uint32_t finish = myTick + del;
	while ( myTick < finish)
	{}
}

