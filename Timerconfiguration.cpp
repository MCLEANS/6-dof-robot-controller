/*
 * Timerconfiguration.cpp
 *
 *  Created on: Mar 30, 2020
 *      Author: MCLEANS
 */

#include "Timerconfiguration.h"

namespace custom_libraries {

Timer_configuration::Timer_configuration(TIM_TypeDef *TIMER, uint16_t prescaler_value, uint16_t auto_reload_value) : TIMER(TIMER),
																												prescaler_value(prescaler_value),
																												auto_reload_value(auto_reload_value){

		//Enable Timer RCC
		if(TIMER == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		if(TIMER == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		if(TIMER == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		if(TIMER == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		if(TIMER == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
		if(TIMER == TIM6) RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		if(TIMER == TIM7) RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
		if(TIMER == TIM8) RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
		if(TIMER == TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
		if(TIMER == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
		if(TIMER == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
		if(TIMER == TIM12) RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
		if(TIMER == TIM13) RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
		if(TIMER == TIM14) RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

		TIMER->PSC = this->prescaler_value;
		TIMER->ARR = this-> auto_reload_value;
}

void Timer_configuration::set_prescaler(uint16_t prescaler_value){
	this->prescaler_value = prescaler_value;

	if(this->prescaler_value != TIMER->PSC) TIMER->PSC = this->prescaler_value;
}

uint16_t Timer_configuration::get_prescaler(void) const{
	return this->prescaler_value;
}

void Timer_configuration::set_auto_reload_value(uint16_t auto_reload_value){
	this->auto_reload_value = auto_reload_value;

	if(this->auto_reload_value != TIMER->ARR) TIMER->ARR = this->auto_reload_value;
}

uint16_t Timer_configuration::get_auto_reload_value(void) const{
	return this->auto_reload_value;
}

void Timer_configuration::initialize(){

	//Initiate update event
	TIMER->EGR |= TIM_EGR_UG;
	//Enable update interrupt
	TIMER->DIER |= TIM_DIER_UIE;
	//only timer overflow generates update event
	TIMER->CR1 |= TIM_CR1_URS;
	//Enable counter
	TIMER->CR1 |= TIM_CR1_CEN;

}

Timer_configuration::~Timer_configuration() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
