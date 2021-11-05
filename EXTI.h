/*
 * EXTI.h
 *
 *  Created on: Apr 19, 2020
 *      Author: MCLEANS
 */

#ifndef EXTI_H_
#define EXTI_H_

#include "stm32f4xx.h"

namespace custom_libraries {

enum edge{
	FALLING,
	RISING
};

class _EXTI {
private:
	GPIO_TypeDef *GPIO;
	uint8_t PIN;
	edge interrupt_edge;

private:
public:
public:
	_EXTI(GPIO_TypeDef *GPIO,
			uint8_t PIN,
			edge interrupt_edge);
	void initialize();
	~_EXTI();
};

} /* namespace custom_libraries */

#endif /* EXTI_H_ */
