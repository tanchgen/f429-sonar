/*
 * gpio.c
 *
 *  Created on: 26 сент. 2017 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32f4xx.h"
#include "gpio.h"

void tim3GpioInit( void ){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	// TODO: ------- Для теста - в рабочем варианте надо убрать ----------------
	// Управляющий пин PD4
	GPIOD->MODER |= (0x1 << (4 * 2));
	GPIOD->OSPEEDR |= (0x2 << (4 * 2));
	GPIOD->BSRRL |= GPIO_Pin_4;
	// Выход TIM3_CH1 PB4
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= (0x2 << (4 * 2));
	GPIOB->OSPEEDR |= (0x2 << (4 * 2));
	// Альтернативная функция PB4 - AF2
	GPIOB->AFR[0] |= 0x2 << (4 * 4);

	// Настройка TIM_ETR

	// Альтернативная функция PD2
	GPIOD->MODER |= (0x2 << (2 * 2));
	GPIOD->OSPEEDR |= (0x2 << (2 * 2));
	// Альтернативная функция PD2 - AF2
	GPIOD->AFR[0] |= 0x2 << (2 * 4);

}

void tim2GpioInit( void ){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// TODO: ------- Для теста - в рабочем варианте надо убрать ----------------
	// Управляющий пин PA5
	// Выход TIM2_CH1 PA5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (0x2 << (5 * 2));
	GPIOA->OSPEEDR |= (0x2 << (5 * 2));
//	GPIOA->BSRRL |= GPIO_Pin_5;
	// Альтернативная функция PA5 - AF1
	GPIOA->AFR[0] |= 0x1 << (5 * 4);

	// Управляющий пин PB3
	// Выход TIM2_CH2 PB3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= (0x2 << (3 * 2));
	GPIOB->OSPEEDR |= (0x2 << (3 * 2));
//	GPIOA->BSRRL |= GPIO_Pin_3;
	// Альтернативная функция PB3 - AF1
	GPIOB->AFR[0] |= 0x1 << (3 * 4);

}

void tim1GpioInit( void ){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// Выход PE8 - TIM1_CH1N, PE9 - TIM1_CH1
	GPIOE->MODER |= (0x2 << (8 * 2)) | (0x2 << (9 * 2));
	GPIOE->OSPEEDR |= (0x3 << (8 * 2)) | (0x3 << (9 * 2));
//	GPIOE->BSRRL |= GPIO_Pin_8 | GPIO_Pin_9;
	// Альтернативная функция PE8, PE9 - AF1
	GPIOE->AFR[1] |= 0x1;		              // PA8
	GPIOE->AFR[1] |= (0x1 << 4);		      // PA9

	// Break input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// Вывод PE15 - TIM1_BRIN
	GPIOE->MODER |= (0x2 << (15 * 2));
	GPIOE->OSPEEDR |= (0x3 << (15 * 2));
	// Альтернативная функция PE15 - AF1
	GPIOE->AFR[1] |= (0x1 << ((15-8) * 4));		// PE15

}

void tim8GpioInit( void ){

	// Выход TIM8_CH1 PC6
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= (0x2 << (6 * 2));
	GPIOC->OSPEEDR |= (0x2 << (6 * 2));
//	GPIOA->BSRRL |= GPIO_Pin_6;
	// Альтернативная функция PC6 - AF3
	GPIOC->AFR[0] |= 0x3 << (6 * 4);

	// Выход PA6 - TIM8_BKIN
	GPIOA->MODER |= (0x2 << (6 * 2));
	GPIOA->OSPEEDR |= (0x3 << (6 * 2));
//	GPIOA->BSRRL |= GPIO_Pin_3;
  // Альтернативная функция PA6 - AF3 = TIM8_BKIN
	GPIOA->AFR[0] |= (0x3 << (6 * 4));		// PC6

}

void tim5GpioInit( void ){

	// Выход TIM5_CH1 PA0
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= 0x2;
	GPIOA->OSPEEDR |= 0x2;
//	GPIOA->BSRRL |= GPIO_Pin_0;
	// Альтернативная функция PA0 - AF2
	GPIOA->AFR[0] |= 0x2;
}

// TIM5 - DAC conversion trigger
void dacGpioInit( void ){
	// Выход DAC_CH1 PA4
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->PUPDR &= ~(0x3 << (4 * 2));
	GPIOA->MODER |= (0x3 << (4 * 2));
}

// TIM4_CH4 - ADC conversion trigger
// For debug Pin Output - PB9
void tim4GpioInit( void ){

	// Выход TIM4_CH4 PB9
	// Break input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// Вывод PB9 - TIM4_CH4
	GPIOB->MODER |= (0x2 << (9 * 2));
	GPIOB->OSPEEDR |= (0x3 << (9 * 2));
	// Альтернативная функция PB9 - AF2
	GPIOB->AFR[1] |= (0x2 << ((9-8) * 4));		// PB9
}

void adcGpioInit( void ){
  // ****** Configure ADC1 Channel3 - PA3 pin as analog input ****************

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->PUPDR &= ~(0x3 << (3 * 2));
	GPIOA->MODER |= (0x3 << (3 * 2));
}
