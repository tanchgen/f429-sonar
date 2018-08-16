/*
 * dac.c
 *
 *  Created on: 06 нояб. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32f4xx.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_adc.h"

#include "gpio.h"
#include "my_time.h"
#include "dac_adc.h"

uint16_t dacData[DAC_SAMPLE_NUM];
uint16_t adcData[ADC_SAMPLE_NUM];
volatile uint8_t fullAdcDma = 0;
uint8_t varuLevel = 0;

uint16_t GetVaruValueFromGainValue(uint16_t gain_value);
uint16_t GetGainValueFromVaruValue(uint16_t varu_value);

void dacInit( void ){
	dacGpioInit();

	// Инициализация массива данных DAC
	dacDataInit( varuLevel, DAC_SAMPLE_NUM );

	// Инициализация DAC
  DMA_InitTypeDef DMA_InitStructure;

  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  // Trigger select TIM5,
  DAC->CR = DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1;

  // DMA1_Stream6 (DAC1, Stream5 - DAC2) channel7 configuration **************************************/
  DMA_DeInit(DMA1_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->DHR12R1);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dacData;
  DMA_InitStructure.DMA_BufferSize = DAC_SAMPLE_NUM;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);

  // Enable DMA1_Stream5
  DMA_Cmd(DMA1_Stream5, ENABLE);
//  DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);
//  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

//  NVIC_EnableIRQ( DMA1_Stream5_IRQn );
//  NVIC_SetPriority( DMA1_Stream5_IRQn, 1 );

  // Enable DAC Channel1, Enable DMA for DAC Channel1
  DAC->CR |= DAC_CR_EN1 | DAC_CR_DMAEN1;


//  DAC->CR |= 0xC << 8;
//  DAC->CR |= 0x2 << 6;
#if 0
  while(1){
  	for( uint16_t i=0; i < DAC_SAMPLE_NUM; i++ ){
   		DAC->DHR12R1 = dacData[i];
  		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
  		myDelay(1);
  	}
 // 	myDelay(500);
  }
#endif

}

void dacDataInit( uint8_t vLevel, uint16_t smplNum ){
//	uint16_t k = 40960/smplNum;
//
//	for(uint16_t i = 0; i < smplNum; i++ ){
//		*pvaru++ = (k * i)/10;
//	}
  // Значение которым необходимо заполнить таблицу вару
  u16 varuTableValue = GetVaruValueFromGainValue(vLevel);
  // Записываем неизменное значение ВАРУ
  for(uint16_t i = 0; i < smplNum; i++ ){
    dacData[i] = varuTableValue;
  }

}

void adcInit( void ){
	adcGpioInit();

  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcData;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_NUM;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  DMA_ITConfig(DMA2_Stream0, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
  NVIC_EnableIRQ( DMA2_Stream0_IRQn );
  NVIC_SetPriority( DMA2_Stream0_IRQn, 1 );

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel13 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_3Cycles);

  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

//  ADC1->CR1 |= ADC_CR1_EOCIE;
//  NVIC_EnableIRQ( ADC_IRQn );
//  NVIC_SetPriority( ADC_IRQn, 1 );



  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

}

// Перезапуск DAC для очередного цикла
void dacReset( void ){
	uint8_t rc;
	uint32_t tmpcr;

	DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	DMA1->HIFCR = 0;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
	// ---- Загружаем вручную нулевое значение с помощью DMA ---
	// Сохраним
	tmpcr = DAC->CR;
	// Запускаем программный тригер
  DAC->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0;
	// Перезапуск DAC для очередного цикла
  DAC->DHR12R1 = 0;
  // Запускаем
  DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
  while( (rc = DAC->SWTRIGR & DAC_SWTRIGR_SWTRIG1) != 0 )
  {}
  // Восстанавливаем CR
  DAC->CR = tmpcr;
}

void adcProcess( DMA_Stream_TypeDef * DMA_Streamx ){
	__aligned(4) uint16_t * padc = adcData;
	uint16_t len;

	len = ADC_SAMPLE_NUM - ((DMA_Streamx->NDTR == ADC_SAMPLE_NUM)? 0: DMA_Streamx->NDTR);

	if( fullAdcDma == SET ){
		padc = adcData + ADC_SAMPLE_NUM/2;
		len -= ADC_SAMPLE_NUM/2;
	}

	// Запускаем DMA в PBUF для UDP
	udpTransfer( (uint32_t*)padc, len );

	// В случае выборки по таймеру окончания Главного цикла будем выбирать из следующей половины
	fullAdcDma = !fullAdcDma;
}

uint16_t GetVaruValueFromGainValue(uint16_t gain_value) {
  uint16_t varu_value;
  if( (gain_value > 0) && (gain_value < 14) ) {
      varu_value = 307 * gain_value + (gain_value - 1);
  }
  else if( gain_value == 14 ){
      varu_value = 4095;
  }
  else {
      varu_value = 1;
  }

  return varu_value;
}

uint16_t GetGainValueFromVaruValue(uint16_t varu_value){
  uint16_t gain_value;

  if(varu_value == 4095) {
    gain_value = 14;
  }
  else if( (varu_value <= 1) || (varu_value > 4095)){
    gain_value = 0;
  }
  else {
    gain_value = varu_value / 307;
  }
  return gain_value;
}
