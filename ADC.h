/*
 * ADC.h
 *
 *  Created on: Apr 9, 2020
 *      Author: MCLEANS
 *
 *      This Library expects the user to implement their own interrupt service routine
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"

namespace custom_libraries {

enum Sampling_rate{
	SLOW,
	MEDIUM,
	FAST
};

enum ADC_channel{
	ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15
};

class _ADC{
private:
	ADC_TypeDef *ADC_;
	GPIO_TypeDef *GPIO;
	uint8_t PIN;
	ADC_channel channel;
	Sampling_rate sample_rate;



private:
	void delay_ms(uint32_t duration);

public:
	uint32_t count = 0;

public:
	_ADC(ADC_TypeDef *ADC_,GPIO_TypeDef *GPIO,uint8_t PIN,ADC_channel channel,Sampling_rate sample_rate);
	void initialize();
	~_ADC();

};

} /* namespace custom_libraries */

#endif /* ADC_H_ */
