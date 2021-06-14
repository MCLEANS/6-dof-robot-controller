/*
 * ADC.cpp
 *
 *  Created on: Apr 9, 2020
 *      Author: MCLEANS
 */

#include "ADC.h"

namespace custom_libraries {

_ADC::_ADC(ADC_TypeDef *ADC_,GPIO_TypeDef *GPIO,uint8_t PIN,ADC_channel channel,Sampling_rate sample_rate):ADC_(ADC_),
																										GPIO(GPIO),
																										PIN(PIN),
																										channel(channel),
																										sample_rate(sample_rate){

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

	//Enable ADC_RCC
	if(ADC_ == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	if(ADC_ == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	if(ADC_ == ADC3) RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

	//Set GPIO to analog mode
	GPIO->MODER |= (1<<((PIN * 2)));
	GPIO->MODER |= (1<<((PIN * 2)+1));

	//Set ADC prescaler
	//This is done in the ADC common control register
	ADC->CCR |= ADC_CCR_ADCPRE_0; //prescaler of 4 to set clock to 21MHz (must be < 30MHz)

	//enable ADC end of conversion
	ADC_->CR1 |= ADC_CR1_EOCIE;

	//Set the sampling rate
	switch(sample_rate){
		case SLOW:
			if(channel < 10){
				ADC_->SMPR2 |= (1 <<((channel*3)));
				ADC_->SMPR2 |= (1 <<((channel*3)+1));
				ADC_->SMPR2 |= (1 <<((channel*3)+2));
			}
			else{
				ADC_->SMPR1 |= (1 <<(((channel-10)*3)));
				ADC_->SMPR1 |= (1 <<(((channel-10)*3)+1));
				ADC_->SMPR1 |= (1 <<(((channel-10)*3)+2));
			}
			break;
		case MEDIUM:
			if(channel < 10){
				ADC_->SMPR2 &= ~(1 <<((channel*3)));
				ADC_->SMPR2 &= ~(1 <<((channel*3)+1));
				ADC_->SMPR2 |= (1 <<((channel*3)+2));
			}
			else{
				ADC_->SMPR1 &= ~(1 <<(((channel-10)*3)));
				ADC_->SMPR1 &= ~(1 <<(((channel-10)*3)+1));
				ADC_->SMPR1 |= (1 <<(((channel-10)*3)+2));
			}
			break;
		case FAST:
			if(channel < 10){
				ADC_->SMPR2 &= ~(1 <<((channel*3)));
				ADC_->SMPR2 &= ~(1 <<((channel*3)+1));
				ADC_->SMPR2 &= ~(1 <<((channel*3)+2));
			}
			else{
				ADC_->SMPR1 &= ~(1 <<(((channel-10)*3)));
				ADC_->SMPR1 &= ~(1 <<(((channel-10)*3)+1));
				ADC_->SMPR1 &= ~(1 <<(((channel-10)*3)+2));
			}
			break;
		default:
			break;

	}


	//Set number of channels
	//Set all its registers to zero signaling one conversion
	ADC_->SQR1 &= ~ADC_SQR1_L;

	//Set channel sequence
	switch(channel){
		case ch0:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch1:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch2:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch3:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch4:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch5:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch6:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch7:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch8:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch9:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch10:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch11:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch12:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch13:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch14:
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		case ch15:
			ADC_->SQR3 |= ADC_SQR3_SQ1_0;
			ADC_->SQR3 |= ADC_SQR3_SQ1_1;
			ADC_->SQR3 |= ADC_SQR3_SQ1_2;
			ADC_->SQR3 |= ADC_SQR3_SQ1_3;
			ADC_->SQR3 &= ~ADC_SQR3_SQ1_4;
			break;
		default:
			break;
	}
}

void _ADC::delay_ms(uint32_t duration){
	this->count = 0;
	while(this->count < duration){}
}

void _ADC::initialize(){
	//Enable ADC and set to continuous mode
	//This first enabling wakes it up from sleep
	ADC_->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;

	//delay time Tstab as stated in reference manual
	delay_ms(2);

	//Enable ADC again to start
	//The second enable actually enables the ADC
	ADC_->CR2 |= ADC_CR2_ADON;

	delay_ms(2);
	//start first ADC conversion
	ADC_->CR2 |= ADC_CR2_SWSTART;
}

_ADC::~_ADC() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
