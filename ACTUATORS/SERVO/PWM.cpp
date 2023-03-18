/*
 * PWM.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: MCLEANS
 */

#include "PWM.h"

namespace custom_libraries {

PWM::PWM(TIM_TypeDef *TIMER,
		channel input_channel,
		GPIO_TypeDef *PORT,
		uint8_t PIN,
		alternate_function pin_function,
		uint16_t prescaler,
		uint16_t auto_reload_value ):TIMER(TIMER),
									input_channel(input_channel),
									PORT(PORT),
									PIN(PIN),
									pin_function(pin_function),
									prescaler(prescaler),
									auto_reload_value(auto_reload_value)
										{
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



	//Enable GPIO RCC
	if(PORT == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if(PORT == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if(PORT == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if(PORT == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if(PORT == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	if(PORT == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	if(PORT == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	if(PORT == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	if(PORT == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

	//Set pin to alternate function mode
	PORT->MODER &= ~(1 << (PIN*2));
	PORT->MODER |= (1 << ((PIN*2)+1));

	set_alternate_function(pin_function);

	TIMER->PSC = this->prescaler;
	TIMER->ARR = this-> auto_reload_value;


}

void PWM::set_prescaler(uint16_t prescaler){

	if(prescaler != this->prescaler){
		this->prescaler = prescaler;
		TIMER->PSC = this->prescaler;
	}


}

uint16_t PWM::get_prescaler(void)const{
	return this->prescaler;

}

void PWM::set_auto_reload_value(uint16_t auto_reload_value){

	if(auto_reload_value != this->auto_reload_value) {
		this->auto_reload_value = auto_reload_value;
		TIMER->ARR = this->auto_reload_value;
	}


}

uint16_t PWM::get_auto_reload_value(void) const{
	return this->auto_reload_value;
}

void PWM::set_duty_cycle(uint16_t duty_cycle){

	if(duty_cycle != this->duty_cycle){
		this->duty_cycle = duty_cycle;

		if(input_channel == channel1) TIMER->CCR1 = this->duty_cycle;
		if(input_channel == channel2) TIMER->CCR2 = this->duty_cycle;
		if(input_channel == channel3) TIMER->CCR3 = this->duty_cycle;
		if(input_channel == channel4) TIMER->CCR4 = this->duty_cycle;
	}

}

uint16_t PWM::get_duty_cycle(void) const{
	return this->duty_cycle;
}

//Check from the micro-controller datasheet to find out the specific alternate function
void PWM::set_alternate_function(alternate_function pin_alternate_function){
	if(pin_alternate_function == AF0) {
		if(PIN < 8){
			PORT->AFR[0] &= ~(1<<(PIN*4));
			PORT->AFR[0] &= ~(1<<((PIN*4)+1));
			PORT->AFR[0] &= ~(1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
		else{
			PORT->AFR[1] &= ~(1<<((PIN-8)*4));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+1));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+2));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+3));
		}
	}

	if(pin_alternate_function == AF1){
		if(PIN < 8){
			PORT->AFR[0] |= (1<<(PIN*4));
			PORT->AFR[0] &= ~(1<<((PIN*4)+1));
			PORT->AFR[0] &= ~(1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
		else{
			PORT->AFR[1] |= (1<<((PIN-8)*4));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+1));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+2));
			PORT->AFR[1] &= ~(1<<(((PIN-8)*4)+3));

		}
	}
	if(pin_alternate_function == AF2){

		if(PIN < 8){
			PORT->AFR[0] &= ~(1<<(PIN*4));
			PORT->AFR[0] |= (1<<((PIN*4)+1));
			PORT->AFR[0] &= ~(1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
		else{
			PORT->AFR[1] &= ~(1<<(PIN*4));
			PORT->AFR[0] |= (1<<((PIN*4)+1));
			PORT->AFR[0] &= ~(1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}

	}
	if(pin_alternate_function == AF3) {
			PORT->AFR[0] |= (1<<(PIN*4));
			PORT->AFR[0] |= (1<<((PIN*4)+1));
			PORT->AFR[0] &= ~(1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF4) {
			PORT->AFR[0] &= ~(1<<(PIN*4));
			PORT->AFR[0] &= ~(1<<((PIN*4)+1));
			PORT->AFR[0] |= (1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF5) {
			PORT->AFR[0] |= (1<<(PIN*4));
			PORT->AFR[0] &= ~(1<<((PIN*4)+1));
			PORT->AFR[0] |= (1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF6) {
			PORT->AFR[0] &= ~(1<<(PIN*4));
			PORT->AFR[0] |= (1<<((PIN*4)+1));
			PORT->AFR[0] |= (1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF7) {
			PORT->AFR[0] |= (1<<(PIN*4));
			PORT->AFR[0] |= (1<<((PIN*4)+1));
			PORT->AFR[0] |= (1<<((PIN*4)+2));
			PORT->AFR[0] &= ~(1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF8) {
			PORT->AFR[1] &= ~(1<<(PIN*4));
			PORT->AFR[1] &= ~(1<<((PIN*4)+1));
			PORT->AFR[1] &= ~(1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
		}
	if(pin_alternate_function == AF9) {
			PORT->AFR[1] |= (1<<(PIN*4));
			PORT->AFR[1] &= ~(1<<((PIN*4)+1));
			PORT->AFR[1] &= ~(1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF10) {
			PORT->AFR[1] &= ~(1<<(PIN*4));
			PORT->AFR[1] |= (1<<((PIN*4)+1));
			PORT->AFR[1] &= ~(1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF11) {
			PORT->AFR[1] |= (1<<(PIN*4));
			PORT->AFR[1] |= (1<<((PIN*4)+1));
			PORT->AFR[1] &= ~(1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF12) {
			PORT->AFR[1] &= ~(1<<(PIN*4));
			PORT->AFR[1] &= ~(1<<((PIN*4)+1));
			PORT->AFR[1] |= (1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF13) {
			PORT->AFR[1] |= (1<<(PIN*4));
			PORT->AFR[1] &= ~(1<<((PIN*4)+1));
			PORT->AFR[1] |= (1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF14) {
			PORT->AFR[1] &= ~(1<<(PIN*4));
			PORT->AFR[1] |= (1<<((PIN*4)+1));
			PORT->AFR[1] |= (1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}
	if(pin_alternate_function == AF15) {
			PORT->AFR[1] |= (1<<(PIN*4));
			PORT->AFR[1] |= (1<<((PIN*4)+1));
			PORT->AFR[1] |= (1<<((PIN*4)+2));
			PORT->AFR[1] |= (1<<((PIN*4)+3));
			}

}

void PWM::begin(){
	//set to PWM Mode 1
	if(input_channel == channel1){
		TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
		TIMER->CCMR1 |= TIM_CCMR1_OC1M_1;
		TIMER->CCMR1 |= TIM_CCMR1_OC1M_2;
		//Set output compare preload enable
		TIMER->CCMR1 |= TIM_CCMR1_OC1PE;
		//Enable capture/compare output
		TIMER->CCER |= TIM_CCER_CC1E;
	}

	if(input_channel == channel2){
		TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
		TIMER->CCMR1 |= TIM_CCMR1_OC2M_1;
		TIMER->CCMR1 |= TIM_CCMR1_OC2M_2;
		//Set output compare preload enable
		TIMER->CCMR1 |= TIM_CCMR1_OC2PE;
		//Enable capture/compare output
		TIMER->CCER |= TIM_CCER_CC2E;
	}

	if(input_channel == channel3){
		TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
		TIMER->CCMR2 |= TIM_CCMR2_OC3M_1;
		TIMER->CCMR2 |= TIM_CCMR2_OC3M_2;
		//Set output compare preload enable
		TIMER->CCMR2 |= TIM_CCMR2_OC3PE;
		//Enable capture/compare output
		TIMER->CCER |= TIM_CCER_CC3E;
	}

	if(input_channel == channel4){
		TIMER->CCMR2 &= ~(TIM_CCMR2_OC4M_0);
		TIMER->CCMR2 |= TIM_CCMR2_OC4M_1;
		TIMER->CCMR2 |= TIM_CCMR2_OC4M_2;
		//Set output compare preload enable
		TIMER->CCMR2 |= TIM_CCMR2_OC4PE;
		//Enable capture/compare output
		TIMER->CCER |= TIM_CCER_CC4E;
	}

	//set main Output Enable
	TIMER->BDTR |= TIM_BDTR_MOE;
	//Enable the UG bit to update preload register
	TIMER->EGR |= TIM_EGR_UG;
	//Enable timer
	TIMER->CR1 |= TIM_CR1_CEN;

}


PWM::~PWM() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
