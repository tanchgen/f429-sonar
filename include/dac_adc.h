/*
 * dac.h
 *
 *  Created on: 06 нояб. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef DAC_H_
#define DAC_H_

//#include "eth.h"

#define DAC_SAMPLE_NUM		4096
#define ADC_SAMPLE_NUM		1472     // Количество выборок АЦП = размер UDP-пакета

extern uint16_t adcData[];

void dacInit( void );
void dacDataInit( uint16_t *pvaru, uint16_t smplNum );
void adcInit( void );
void dacReset( void );
void adcProcess( DMA_Stream_TypeDef * DMA_Streamx );

#endif /* DAC_H_ */
