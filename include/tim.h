/*
 * tim.h
 *
 *  Created on: 25 сент. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef TIM_H_
#define TIM_H_

#include "tcp_srv.h"


enum {
  MAINMODE_CYCLE =	0,			// Режим работы: 0 - Работа циклическая
  MAINMODE_EXT              // Режим работы: 1- разовый цикл от внешнего запуска
};

#define TMAIN				1200		// миллисекунд
#define TGEN				400			// Длительность работы излучателя в микросекундах
#define KPD					90			// Мощность передатчика излучателя
#define FGEN				300			// Частота генерации излучателя в кГц
#define FADC				192102	// Частота преобразования АЦП
#define TDAC				(1000)	// 500мс - Минимальный период главного цикла, 2мс - макс. SW-обработка)

#define TIM5_FREQ		(9e6)
// структура временных интервалов таймеров

typedef struct {
	uint32_t mainMode;		// Режим работы: 0 - Работа циклическая, 1- разовый цикл от внешнего запуска
	uint32_t T1Main;			// Период первого таймера главного цикла сонара
	uint32_t tGen;			  // Длительность генерации импульсов излучателя
	uint32_t fgen;				// Частота излучателя
	uint32_t tDt;					// Длительность задержек Dead-time ШИМ и tPower-ЩИМ
	uint32_t kpd;					// Мощность передатчика излучателя
	uint32_t fAdc;				// Частота преобразования ADC
	uint32_t TDac;				// Период DAC
	eOpMode opMode;       // Главный режим: 0 - Выкл, 1- Передача-Прием, 2- Прием
} tTims;

// ------ Период главного цикла -----------
// tmn - мсек
void tMainSetup( void );
void mainModeSet( void );

// ------ Длительность импульса вкл Энергии излучателя
// tpwr = кол-во импульсов излучателя
void tPwrSetup( void );
// ------ Длительность задержек Dead-time ШИМ и tPower-ЩИМ ( нс ) ---------
void tDtSetup( void );
// ------ Частота излучателя -----------
// fgen - частота в кГц
void fGenSetup( void);
void tGenSetup( void );
// ------- ЦАП - АЦП ----------
void tDacSetup( void );
void tAdcSetup( void );

void tInit( void );
void tim8Init( void );

void tim3Init( void );
void tim2Init( void );
void tim1Init( void );
void tim5Init( void );
void tim4Init( void );

void mainTimStart( void );
void mainTimStop( void );
void timPrestart( void );

void firstSwProcess( void );
void opModeSetup( eOpMode mode );

#endif /* TIM_H_ */
