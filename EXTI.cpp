/*
 * EXTI.cpp
 *
 *  Created on: Apr 19, 2020
 *      Author: MCLEANS
 */

#include "EXTI.h"

namespace custom_libraries {

_EXTI::_EXTI(GPIO_TypeDef *GPIO,
			uint8_t PIN,
			edge interrupt_edge):GPIO(GPIO),
								PIN(PIN),
								interrupt_edge(interrupt_edge){
	//enable SYSCFG RCC
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//Enable GPIO_RCC
	if(GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if(GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if(GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if(GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if(GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	if(GPIO == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	if(GPIO == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	if(GPIO == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	if(GPIO == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
	//select actual ext_int pin in interrupt system configuration register.
	if(PIN < 4 ){
		if(GPIO == GPIOA){
			SYSCFG->EXTICR[0] &= ~(1<<(PIN*4));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
		if(GPIO == GPIOB){
			SYSCFG->EXTICR[0] |= (1<<(PIN*4));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
		if(GPIO == GPIOC){
			SYSCFG->EXTICR[0] &= ~(1<<(PIN*4));
			SYSCFG->EXTICR[0] |= (1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
		if(GPIO == GPIOD){
			SYSCFG->EXTICR[0] |= (1<<(PIN*4));
			SYSCFG->EXTICR[0] |= (1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
		if(GPIO == GPIOE){
			SYSCFG->EXTICR[0] &= ~(1<<(PIN*4));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] |= (1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
		if(GPIO == GPIOF){
			SYSCFG->EXTICR[0] |= (1<<(PIN*4));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+1));
			SYSCFG->EXTICR[0] |= (1<<((PIN*4)+2));
			SYSCFG->EXTICR[0] &= ~(1<<((PIN*4)+3));
		}
}

	if(PIN > 3 and PIN < 8){
		if(GPIO == GPIOA){
			SYSCFG->EXTICR[1] &= ~(1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
		if(GPIO == GPIOB){
			SYSCFG->EXTICR[1] |= (1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
		if(GPIO == GPIOC){
			SYSCFG->EXTICR[1] &= ~(1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] |= (1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
		if(GPIO == GPIOD) {
			SYSCFG->EXTICR[1] |= (1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] |= (1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
		if(GPIO == GPIOE){
			SYSCFG->EXTICR[1] &= ~(1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] |= (1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
		if(GPIO == GPIOF){
			SYSCFG->EXTICR[1] |= (1<<((PIN-4)*4));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+1));
			SYSCFG->EXTICR[1] |= (1<<(((PIN-4)*4)+2));
			SYSCFG->EXTICR[1] &= ~(1<<(((PIN-4)*4)+3));
		}
	}

	if(PIN > 8 and PIN <12){
		if(GPIO == GPIOA){
			SYSCFG->EXTICR[2] &= ~(1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
		if(GPIO == GPIOB){
			SYSCFG->EXTICR[2] |= (1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
		if(GPIO == GPIOC) {
			SYSCFG->EXTICR[2] &= ~(1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] |= (1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
		if(GPIO == GPIOD) {
			SYSCFG->EXTICR[2] |= (1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] |= (1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
		if(GPIO == GPIOE){
			SYSCFG->EXTICR[2] &= ~(1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] |= (1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
		if(GPIO == GPIOF) {
			SYSCFG->EXTICR[2] |= (1<<((PIN-8)*4));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+1));
			SYSCFG->EXTICR[2] |= (1<<(((PIN-8)*4)+2));
			SYSCFG->EXTICR[2] &= ~(1<<(((PIN-8)*4)+3));
		}
	}

	if(PIN > 11 and PIN < 16){
		if(GPIO == GPIOA){
			SYSCFG->EXTICR[3] &= ~(1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}
		if(GPIO == GPIOB) {
			SYSCFG->EXTICR[3] |= (1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}
		if(GPIO == GPIOC) {
			SYSCFG->EXTICR[3] &= ~(1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] |= (1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}
		if(GPIO == GPIOD){
			SYSCFG->EXTICR[3] |= (1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] |= (1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}
		if(GPIO == GPIOE){
			SYSCFG->EXTICR[3] &= ~(1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] |= (1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}
		if(GPIO == GPIOF){
			SYSCFG->EXTICR[3] |= (1<<((PIN-12)*4));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+1));
			SYSCFG->EXTICR[3] |= (1<<(((PIN-12)*4)+2));
			SYSCFG->EXTICR[3] &= ~(1<<(((PIN-12)*4)+3));
		}

	}
}

void _EXTI::initialize(){
	//Unmask the interrupt
	EXTI->IMR |= (1<<PIN);
	//set whether falling edge or rising edge
	if(interrupt_edge == FALLING){
		EXTI->FTSR |= (1<<PIN);
	}
	else if(interrupt_edge == RISING){
		EXTI->RTSR |= (1<<PIN);
	}
	//set interrupt line to input pull_up
	GPIO->MODER &= ~(1<<(PIN*2));
	GPIO->MODER &= ~(1<<((PIN*2)+1));
	GPIO->PUPDR |= (1<<(PIN*2));
	GPIO->PUPDR &= ~(1<<((PIN*2)+1));
}

_EXTI::~_EXTI() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
