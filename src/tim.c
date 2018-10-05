/*
 * tim.c
 *
 *  Created on: 25 сент. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32f4xx.h"

#include "my_time.h"
#include "gpio.h"
#include "dac_adc.h"
#include "tim.h"

tTims tims;
// структура Предыдущих значений временных интервалов таймеров
tTims timsPrev;

uint32_t countGen;		// Количество импульсов генератора в пачке импульса излучателя

void opModeSetup( eOpMode mode );
// ============== Функции установки Периоов и длительностей для таймеров ========

void tInit( void ){

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;

  // структура временных интервалов таймеров
	// и сразу заполняем предыдущие значения

	tims.T1Main = TMAIN;			// Период первого таймера главного цикла сонара
	timsPrev.T1Main = TMAIN;
	tims.mainMode = MAINMODE_CYCLE;
//	tims.tSw = 0;						// Стартовый импульс (задержка SW)
//	timsPrev.tSw = 0;
	tims.tGen = TGEN;			  	// Длительность генерации импульсов излучателя (в мкс)
	timsPrev.tGen = TGEN;
	tims.fgen	= FGEN;					// Частота излучателя
	timsPrev.fgen	= FGEN;
	tims.kpd = KPD;
	timsPrev.kpd = KPD;
	// Период преобразований АЦП
	tims.fAdc = FADC;
	// За цикл ЦАП работает 2048 раз
	tims.TDac = TDAC;					// Длительность работы DAC
	timsPrev.TDac = TDAC;
	tims.opMode = DDS_MODE_OP_CTRL;       // Режим работы: Передача - вкл., Прием - выкл.
  timsPrev.opMode = DDS_MODE_OP_CTRL;

	tMainSetup();
	// Выставляем чатоту генерации
	fGenSetup();
	tDacSetup();
//	tAdcSetup();
}

// ------ Период главного цикла -----------
// tmn - мсек
void tMainSetup( void ){
	uint32_t tmn = tims.T1Main;

	// Добавляем время на начальную и конечную программные обработки (tSw0 и tSw1)
	tmn *= 20;
	tmn--;
	if( TIM3->CCR1 > tmn){
		TIM3->CCR1 = tmn;
	}
	TIM3->ARR = tmn;

	timsPrev.T1Main =	tims.T1Main;

}

void timPrestart( void ){

  // Перезапуск таймера сонара
  TIM4->EGR |= TIM_EGR_UG;
  TIM5->EGR |= TIM_EGR_UG;
  // Перезапуск таймера сонара
  TIM1->EGR |= TIM_EGR_UG;
  // НЕ-Однопульсовый режим
  TIM3->CR1 &= ~TIM_CR1_OPM;
  // Тригерный режим: ResetMode, Запуск от внешнего источника падающий фронт
  TIM3->SMCR &= ~(TIM_SMCR_ETP | TIM_TS_ETRF | TIM_SlaveMode_Trigger);
  TIM3->CR1 |= TIM_CR1_CEN;

}


void mainModeSet( void ){

	if( tims.mainMode ){
		// Режим внешнего запуска главного цикла
		// Однопульсовый режим
		TIM3->CR1 |= TIM_CR1_OPM;
		// Тригерный режим: ResetMode, Запуск от внешнего источника падающий фронт
		TIM3->SMCR |= TIM_SMCR_ETP | TIM_TS_ETRF | TIM_SlaveMode_Trigger;
	}
	else {
		// НЕ-Однопульсовый режим
		TIM3->CR1 &= ~TIM_CR1_OPM;
		// Выключаем тригерный режим: ResetMode, Запуск от внешнего источника падающий фронт
		TIM3->SMCR &= ~(TIM_SMCR_ETP | TIM_TS_ETRF | TIM_SlaveMode_Trigger);
	}
	TIM3->EGR |= TIM_EGR_UG;
	timsPrev.mainMode = tims.mainMode;
}

// ------ Длительность импульса вкл Энергии излучателя
// tpwr = кол-во импульсов излучателя
void tGenSetup( void ){
	fGenSetup();
	timsPrev.tGen = tims.tGen;
}

// ------ Длительность задержек Dead-time ШИМ ( нс ) ---------
void tDtSetup( void ){
	uint32_t tdt;

	uint8_t kpd = (tims.kpd >= 100)? 1 : 100-tims.kpd;
	tims.tDt = (10000/tims.fgen * kpd) / 2;
	if( tims.tDt < 167 ){
		tims.tDt = 167;
	}
	timsPrev.tDt = tims.tDt;
	timsPrev.kpd = tims.kpd;

	// Dead-time кратно 11 нс.

	uint32_t tmp = (2e9L/apb2TimClock);
	tdt = tims.tDt / tmp ;

	if( tdt > 504){
		// Максимально-возможная dead-time
		tdt %= 1008;
		//Dead-time: Tdts = 33нс, dead-time = (32 + tdt) * 16 * Tdts
		tdt /= 16;
		tdt -= 32;
		tdt |= 0xE0;
	}
	else if( tdt > 254 ){
		//Dead-time: Tdts = 33нс, dead-time = (32 + tdt) * 8 * Tdts
		tdt /= 8;
		tdt -= 32;
		tdt |= 0xC0;
	}
	else if( tdt > 127){
		//Dead-time: Tdts = 33нс, dead-time = (64 + tdt) * 2 * Tdts
			tdt /= 2;
			tdt -= 64;
			tdt |= 0x80;
	}
	// Выставляем dead-time TIM1
	TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | tdt;
}

//uint32_t TPwd;			// Период излучателя

// ------ Частота излучателя -----------
// fgen - частота в кГц
inline void fGenSetup( void ){
	uint32_t tmpPsc = (tims.fgen * 1000 * 2);
	uint32_t psc2 = apb2TimClock / tmpPsc;
	uint32_t psc1 = apb1TimClock / tmpPsc;

	// Количество импульсов в пачке
	countGen = tims.tGen * tims.fgen / 1000;
	// Инициализация таймера TIM1 - Генератор излучателя
	TIM1->PSC = psc2 - 1;
	// Выставляем чатоту  TIM2 - Включение PWR
	TIM2->PSC = psc1-1;
	// Длительность PWR
	TIM2->CCR1 = 2 * countGen + 1;

	//Импульс включения энергии
	// Выставляем чатоту TIM8 - Включение выходов TIM1
	uint8_t tmp = apb2TimClock/18e6;
	TIM8->PSC = tmp - 1;

	// Длительность импульса PWR кратно 3.3 мкс
	TIM8->ARR = (countGen * psc2 * 2 ) / tmp ;

	// От частоты генератора зависят другие времена: tDtGen, tGen
	tDtSetup();
//	tDacSetup();
  // Переустановка таймера АЦП
  TIM4->PSC = ((apb1TimClock / tims.fAdc + 1) / 2) - 1;
	timsPrev.fgen = tims.fgen;
}

//uint32_t TAdc;			// Период ADC
//uint32_t TDac;			// Период DAC
void tDacSetup( void ){
	uint32_t tmp;
	uint32_t tmp2;

	uint32_t tmpt;

	// Установка импульса работы ADC-DAC
//	TIM2->CCR2 = (tims.TDac * 1000) / tims.tGen * (2 * countGen);

	// Установка периода TIM-ЦАП: 2048 импульсов, частота предделителя - 9МГц
	tmp = ( (TIM5_FREQ/1000) * tims.TDac)/DAC_SAMPLE_NUM;
	TIM5->ARR = tmp - 1;
	// Установка импульса разрешения ЦАП
	tmp2 = tims.T1Main * 1000;
	tmpt = tmp2 - (tmp2/(TIM3->ARR+1));
	// Выбираем наименьшую длительность
	tmp2 = tims.TDac * 1000;
	tmpt = ( tmp2 > tmpt)? tmpt: tmp2;
	TIM2->CCR2 = (tmpt * 90) / 0x96;
//	TIM2->CCR2 = (((uint32_t)(tmp*(DAC_SAMPLE_NUM+1))/(uint32_t)(TIM5_FREQ/1e6))*(2*countGen))/tims.tGen+1;
	tmp >>= 1;
	TIM5->CCR1 = tmp;
}

// ============================== Установки Таймеров =============================
void tim3Init( void ){
	// Настройка GPIO
	tim3GpioInit();

	// Выставляем чатоту счетчика 2000 Гц

	// Установка нового периода только после окончания старого
	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->PSC = apb1TimClock / 20000 - 1;
	TIM3->CCR1 = 1;

  TIM3->CNT = TIM3->ARR-1;
	//------------ Для тестирования -----------------------
	// Режим вывода PWM1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

	// CC1IF signal is used as trigger output (TRGO)
//	TIM3->CR2 |= TIM_CR2_MMS_1 | TIM_CR2_MMS_0;
//	TIM3->CR2 |= TIM_CR2_MMS_2;

	// Вывод TIM3_CH1
	TIM3->CCER |= TIM_CCER_CC1E;
	// Прерывание для первой обработки
	TIM3->DIER |= TIM_DIER_CC1IE;
	// Прерывание для заключительной обработки
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ( TIM3_IRQn );

}

void tim1Init( void ){
	// Настройка GPIO
	tim1GpioInit();
	// Половина отрицателного импульса 1/4 периода генерации
	TIM1->CCR1 = 1;
	// Половина положителного импульса 1/2 периода генерации
	TIM1->ARR = 2-1;

	// Режим PWM 1
	TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	// Режим center-aligned mode 1
//	TIM1->CR1 = (TIM1->CR1 & ~TIM_CR1_CMS_1) | TIM_CR1_CMS_0;

	// Тригерный режим: ResetMode, Запуск от внутреннего источника ITR1 (TIM2_TRGO) растущий фронт
	TIM1->SMCR |= TIM_TS_ITR1 | TIM_SlaveMode_Reset;

	//Комплементарный выход
//	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;

	TIM1->CR1 |= (TIM1->CR1 & ~TIM_CR1_CKD) | TIM_CR1_CKD_0;
#if 1
	// Break-режим
	TIM1->BDTR |= TIM_BDTR_BKE;
	TIM1->BDTR |= TIM_BDTR_OSSI;
#else
	//Dead-time: Tdts = 33нс, dead-time = 200нс, DTS = 0x6
	uint32_t tmp = TIM1->BDTR;
	tmp &= ~0x7F;
	tmp |= 0x24;
	TIM1->BDTR = tmp;
#endif

	TIM1->CR1 |= TIM_CR1_CEN;

}

void tim2Init( void ){
	/* Таймер запускает:
	 *	TIM2_CH2
	 * 	- таймер TIM4 АЦП приемника,
	 * 	- таймер TIM5 ЦАП РУ усилителя приемника,
	 * 	TIM2_CH1
	 * 	- таймер TIM8 NonBRIN для генерации импульсов излучения
	 */

	// Настройка GPIO
	tim2GpioInit();

	//Импульс включения энергии
	// Длительность импульса PWR кратно 3.3 мкс
	TIM2->ARR = 2000000 - 1;

	// Однопульсовый режим
	TIM2->CR1 |= TIM_CR1_OPM;
	// CC2REF signal is used as trigger output (TRGO)
	TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_2 | TIM_CR2_MMS_0;

	// В прерывании выключаем таймеры TIM4(АЦП) и TIM5(ЦАП)
	TIM2->DIER |= TIM_DIER_CC2IE;
	NVIC_EnableIRQ( TIM2_IRQn );

  TIM2->CNT = TIM2->ARR-2;

#if LOGIC_ANALIZ  // Для логического анализатора
	// Вывод TIM2_CH1
	TIM2->CCER |= TIM_CCER_CC1E;
//	TIM2->CCER |= TIM_CCER_CC1P;
	// Режим вывода PWM1 CH1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

	// Вывод TIM2_CH2
	TIM2->CCER |= TIM_CCER_CC2E;
	// Режим вывода PWM2 CH2
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // | TIM_CCMR1_OC2M_0;
#endif
}



void tim8Init( void ){
	// Настройка GPIO
	tim8GpioInit();

	// Задержка PWR для поппадания dead-time генратора
	TIM8->CCR1 = 1;

#if 0
	// Break-режим
	// Разрешаем автоматическое восстановление MOE
	TIM1->BDTR |= TIM_BDTR_AOE;
//	TIM1->BDTR |= TIM_BDTR_BKP;
	TIM1->BDTR |= TIM_BDTR_BKE;
	TIM1->BDTR |= TIM_BDTR_OSSI;
#endif

#if LOGIC_ANALIZ  // Для логического анализатора
	// Вывод TIM2_CH1
	TIM8->CCER |= TIM_CCER_CC1E;
//	TIM2->CCER |= TIM_CCER_CC1P;
	// Режим вывода PWM1 CH1
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
#endif

	// Однопульсовый режим
	TIM8->CR1 |= TIM_CR1_OPM;
	// Тригерный режим: TriggerMode, Запуск от внутреннего источника ITR1 (TIM2_TRGO) растущий фронт
	TIM8->SMCR |= TIM_TS_ITR1 | TIM_SlaveMode_Trigger;
  // Запрещаем автоматическое восстановление MOE
  TIM8->BDTR = (TIM8->BDTR & ~TIM_BDTR_AOE) | TIM_BDTR_OSSI | TIM_BDTR_MOE;
//  TIM8->CNT = TIM8->ARR-2;
	TIM8->CR1 |= TIM_CR1_CEN;
}

// Таймер ЦАП
void tim5Init( void ){
	// Настройка GPIO
	tim5GpioInit();

	//Импульс включения энергии
	// Длительность импульса PWR кратно 3.3 мкс
	TIM5->PSC = (apb1TimClock/TIM5_FREQ) - 1;

	// UEV signal is used as trigger output (TRGO)
	TIM5->CR2 |= TIM_CR2_MMS_2;
	// Тригерный режим: GatedMode, Запуск от внутреннего источника TIM2 TRGO, растущий фронт
	TIM5->SMCR |= TIM_TS_ITR0 | TIM_SlaveMode_Gated;
	// В режиме "gated" требуется установить CEN
  TIM5->CR1 |= TIM_CR1_CEN;

	//------------ Для тестирования -----------------------
//	TIM5->DIER |= TIM_DIER_UIE;
//	NVIC_EnableIRQ( TIM5_IRQn );

#if LOGIC_ANALIZ  // Для логического анализатора
	// Вывод TIM5_CH1
	TIM5->CCER |= TIM_CCER_CC1E;
//	TIM2->CCER |= TIM_CCER_CC1P;
	// Режим вывода PWM1 CH1
	TIM5->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
#endif
}

// Тиймер АЦП
void tim4Init( void ){
	// Настройка GPIO
	tim4GpioInit();

	TIM4->PSC = ((apb1TimClock / tims.fAdc + 1) / 2) - 1;
	// Установка периода TIM-АЦП: 2/fAdc
	TIM4->ARR = 1;
	TIM4->CCR4 = 1;

	// CC2REF signal is used as trigger output (TRGO)
	TIM4->CR2 |= TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0;
	// Тригерный режим: ResetMode, Запуск от внутреннего источника TIM2 TRGO, растущий фронт
	TIM4->SMCR |= TIM_TS_ITR1 | TIM_SlaveMode_Gated;
	// В режиме "gated" требуется установить CEN
  TIM4->CR1 |= TIM_CR1_CEN;

	//------------ Для тестирования -----------------------
//	TIM4->DIER |= TIM_DIER_UIE;
//	TIM4->DIER |= TIM_DIER_CC4IE;
//	NVIC_EnableIRQ( TIM4_IRQn );

#if LOGIC_ANALIZ  // Для логического анализатора
	// Вывод TIM5_CH1
	TIM4->CCER |= TIM_CCER_CC4E;
//	TIM2->CCER |= TIM_CCER_CC1P;
	// Режим вывода PWM1 CH1
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
#endif
}

void firstSwProcess( void ){
	// Настройки таймеров, если параметры изменились
	if(	tims.T1Main != 	timsPrev.T1Main ){
		// Ограничения: от 500 до 2000 мс
		if( tims.T1Main < 500 ){
			tims.T1Main = 500;
		}
		else if( tims.T1Main > 2000 ){
			tims.T1Main = 2000;
		}

		tMainSetup();
	}
	if (timsPrev.TDac != tims.TDac ){
		tDacSetup();
		timsPrev.TDac = tims.TDac;
	}
	if( tims.tGen != timsPrev.tGen ){
		// Ограничения: от 400 до 5000 импульсов
		if( tims.tGen < 400 ){
			tims.tGen = 400;
		}
		else if( tims.tGen > 5000 ){
			tims.tGen = 5000;
		}
		if( tims.fgen != timsPrev.fgen ){
		  // Проверка нового значения fgen на корректность
      if( tims.fgen < 250 ){
        tims.fgen = 250;
      }
      else if( tims.fgen > 350 ){
        tims.fgen = 350;
      }
		}
		tGenSetup();
	}
	if( timsPrev.kpd != tims.kpd ){
		tDtSetup();
	}
	if( timsPrev.mainMode != tims.mainMode ){
		mainModeSet();
	}

	if(timsPrev.fAdc != tims.fAdc ){
	  // Переустановка таймера ЦАП
	  TIM4->PSC = ((apb1TimClock / tims.fAdc + 1) / 2) - 1;
	  timsPrev.fAdc = tims.fAdc;
	}
	// Переключение режима работы: Стоп, Передача-Прием, Прием
	if(timsPrev.opMode != tims.opMode ){
	  opModeSetup( tims.opMode );
	}

}


void mainTimStart( void ){
// Режим циклического запуска главного цикла
  TIM3->CR1 |= TIM_CR1_CEN;
}

void mainTimStop( void ){
// Режим циклического запуска главного цикла
  TIM3->CR1 &= ~TIM_CR1_CEN;
}

void opModeSetup( eOpMode mode ){
  if( mode == DDS_MODE_STOP){
      // Останавливаем работу ВСЕГО:
      // Не однопульсовый режим
      TIM3->CR1 &= ~TIM_CR1_OPM;
      // Выключаем тригерный режим: ResetMode, Запуск от внешнего источника падающий фронт
      TIM3->SMCR &= ~(TIM_SMCR_ETP | TIM_TS_ETRF | TIM_SlaveMode_Trigger);
      mainTimStop();
  }
  else {
    if( mode == DDS_MODE_OP_CTRL){
      // Выключаем вывод TIM8 - запрещаем вывод
      TIM8->BDTR |= TIM_BDTR_MOE;
    }
    else if( mode == DDS_MODE_RX_CTRL) {
      // Выключаем вывод TIM8 - запрещаем вывод
      TIM8->BDTR &= ~TIM_BDTR_MOE;
    }
    if( timsPrev.opMode == DDS_MODE_STOP ){
      // Восстанавливаем работу главного таймера цикла (TIM3)
      mainModeSet();
      if( tims.mainMode == MAINMODE_CYCLE){
        // Запускаем главный таймер цикла (TIM3)
        mainTimStart();
      }
    }
  }

}
