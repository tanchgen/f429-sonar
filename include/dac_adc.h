/*
 * dac.h
 *
 *  Created on: 06 нояб. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef DAC_H_
#define DAC_H_

#include "eth.h"

#define DAC_SAMPLE_NUM		4096
#define ADC_SAMPLE_NUM		(OUTBUF_SIZE)     // Количество выборок АЦП = размер UDP-пакета

extern uint16_t adcData[];
extern uint8_t varuLevel;

void dacInit( void );
void dacDataInit( uint8_t vLevel, uint16_t smplNum );
void adcInit( void );
void dacReset( void );
void adcProcess( DMA_Stream_TypeDef * DMA_Streamx );
void adcTransfer( uint16_t * data, uint16_t len);

#endif /* DAC_H_ */
