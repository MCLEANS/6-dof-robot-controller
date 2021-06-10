/*
 * GPIO.cpp
 *
 *  Created on: May 4, 2020
 *      Author: MCLEANS
 */

#include "GPIO.h"

namespace custom_libraries {

_GPIO::_GPIO(GPIO_TypeDef *GPIO,uint8_t PIN):GPIO(GPIO),
											PIN(PIN),
											mode(OUTPUT),
											type(OPEN_DRAIN),
											speed(LOW),
											state(FLOATING){
	// TODO Auto-generated constructor stub
	//Enable GPIO RCC
	if(GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if(GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if(GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if(GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if(GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	if(GPIO == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	if(GPIO == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	if(GPIO == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	if(GPIO == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

}

void _GPIO::pin_mode(Mode mode){
	this->mode = mode;

	if(mode == INPUT){
		GPIO->MODER &= ~(1<<(PIN*2));
		GPIO->MODER &= ~(1<<((PIN*2)+1));
	}
	else if(mode == OUTPUT){
		GPIO->MODER |= (1<<(PIN*2));
		GPIO->MODER &= ~(1<<((PIN*2)+1));
	}
}

void _GPIO::output_settings(OUTPUT_Type type, OUTPUT_Speed speed){
	this->type = type;
	this->speed = speed;

	if(type == OPEN_DRAIN){
		GPIO->OTYPER |= (1<<PIN);
	}
	else if(type == PUSH_PULL){
		GPIO->OTYPER &= ~(1<<PIN);
	}
	if(speed == LOW){
		GPIO->OSPEEDR &= ~(1<<(PIN*2));
		GPIO->OSPEEDR &= ~(1<<((PIN*2)+1));
	}
	else if(speed == MEDUIM){
		GPIO->OSPEEDR |= (1<<(PIN*2));
		GPIO->OSPEEDR &= ~(1<<((PIN*2)+1));
	}
	else if(speed == HIGH){
		GPIO->OSPEEDR &= ~(1<<(PIN*2));
		GPIO->OSPEEDR |= (1<<((PIN*2)+1));
	}
	else if(speed == VERY_HIGH){
		GPIO->OSPEEDR |= (1<<(PIN*2));
		GPIO->OSPEEDR |= (1<<((PIN*2)+1));
	}

}

void _GPIO::input_state(State state){
	this->state = state;

	if(state == FLOATING){
		GPIO->PUPDR &= ~(1<<(PIN*2));
		GPIO->PUPDR &= ~(1<<((PIN*2)+1));
	}
	else if(state == PULL_UP){
		GPIO->PUPDR |= (1<<(PIN*2));
		GPIO->PUPDR &= ~(1<<((PIN*2)+1));

	}
	else if(state == PULL_DOWN){
		GPIO->PUPDR &= ~(1<<(PIN*2));
		GPIO->PUPDR |= (1<<((PIN*2)+1));
	}
}

void _GPIO::digital_write(bool value){
	if(value){
		GPIO->ODR |= (1<<PIN);
	}
	if(!value){
		GPIO->ODR &= ~(1<<PIN);
	}
}

bool _GPIO::digital_read(void)const{
	if(GPIO->IDR & (1<<PIN)){
		return true;
	}

	return false;
}
	
void _GPIO::toggle(){
	GPIO->ODR ^= (1 << PIN);
}

_GPIO::~_GPIO() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
