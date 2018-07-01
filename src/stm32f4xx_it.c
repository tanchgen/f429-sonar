/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_adc.h"
#include "my_time.h"
#include "tim.h"
#include "dac_adc.h"
#include "stm32f4xx_it.h"
//#include "eth.h"

extern volatile uint8_t fullAdcDma;

extern uint32_t mainModeCount;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t adcCount = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  // Go to infinite loop when Hard Fault exception occurs
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  // Go to infinite loop when Memory Manage exception occurs
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  // Go to infinite loop when Bus Fault exception occurs
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  // Go to infinite loop when Usage Fault exception occurs
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void PPP_IRQHandler(void)
{
}


// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void) {
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  myTick++;
  // Таймаут для запуска Главного Цикла
  if ( mainModeCount > 1) {
    mainModeCount--;
  }
  //timersHandler();
}
/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles External line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void){
}

// Прерывание датчика Холла - измерителя потока
void EXTI9_5_IRQHandler(void){
}

/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void){
	if (EXTI_GetITStatus(EXTI_Line10)) {
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}


void TIM3_IRQHandler( void ){
	if( (TIM3->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF ){
		TIM3->SR &= ~TIM_SR_CC1IF;
		firstSwProcess();
		// Перезапуск таймера сонара
		TIM2->EGR |= TIM_EGR_UG;
//		TIM4->EGR |= TIM_EGR_UG;
//		TIM5->EGR |= TIM_EGR_UG;
		TIM2->CR1 |= TIM_CR1_CEN;
		// Перезапуск таймера сонара
		TIM1->EGR |= TIM_EGR_UG;
    TIM1->BDTR |= TIM_BDTR_MOE;
	}
	else 	if( (TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF ){
		TIM3->SR &= ~TIM_SR_UIF;
		// Обработка оцифрованных данных
		if(adcCount){
		  adcProcess( DMA2_Stream0 );
		}
		// Перезапуск таймера DAC
		dacReset();
	}
}

void TIM4_IRQHandler( void ){
	if( TIM4->SR & TIM_SR_UIF ){
		TIM4->SR &= ~TIM_SR_UIF;
	}
	else if( TIM4->SR & TIM_SR_CC4IF ){
		TIM4->SR &= ~TIM_SR_CC4IF;
	}
}

void TIM2_IRQHandler( void ){
	TIM2->SR &= ~TIM_SR_CC2IF;
}

void TIM5_IRQHandler( void ){
	if( TIM5->SR & TIM_SR_UIF ){
		TIM5->SR &= ~TIM_SR_UIF;
	}
	else if( TIM5->SR & TIM_SR_CC2IF ){
		TIM5->SR &= ~TIM_SR_CC2IF;
	}
}


void TIM8_CC_IRQHandler(void){
	TIM8->SR &= ~TIM_SR_CC1IF;
}

void TIM8_UP_TIM13_IRQHandler(void){
	TIM8->SR &= ~TIM_SR_UIF;
}

void TIM1_UP_TIM10_IRQHandler(void){
//	TIM1->SR &= ~TIM_SR_UIF;
}

void TIM1_TRG_COM_TIM11_IRQHandler(void){
	TIM1->SR &= ~TIM_SR_TIF;
}

void EXTI0_IRQHandler(void){
//	buttonCount++;
	EXTI_ClearITPendingBit(EXTI_Line0);
}

// DMA для ЦАП
void DMA1_Stream5_IRQHandler(void){
	if(DMA1->HISR & DMA_HISR_TCIF5){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
	}
	if(DMA1->HISR & DMA_HISR_HTIF5){
		DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
	}
}

// DMA для АЦП
void DMA2_Stream0_IRQHandler(void){
	if(DMA2->LISR & DMA_LISR_TCIF0){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		fullAdcDma = SET;
	}
	else if(DMA2->LISR & DMA_LISR_HTIF0){
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		fullAdcDma = RESET;
	}
	else {
	  return;
	}
	// TODO: Пересылка половины блока оцифрованных данных
  adcCount++;
	adcProcess( DMA2_Stream0 );
}

// Отправка данных, переданных DMA в PBUF UDP
void DMA2_Stream3_IRQHandler(void){
  if(DMA2->LISR & DMA_LISR_TCIF3){
    // Данные из АЦП переправлены в PBUF UDP
    DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
  }
  // TODO: Пересылка половины блока оцифрованных данных в сеть
 // udp_send(upcb, outp);
 // pbuf_free(outp);
}

void ADC_IRQHandler( void ){
	adcCount++;
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

void TIM6_DAC_IRQHandler(void){
}
