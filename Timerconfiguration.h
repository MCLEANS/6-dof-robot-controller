/*
 * Timerconfiguration.h
 *
 *  Created on: Mar 30, 2020
 *      Author: MCLEANS
 *
 *      This library expects you to implement your own Interrupt handler
 */

#ifndef TIMERCONFIGURATION_H_
#define TIMERCONFIGURATION_H_

#include "stm32f4xx.h"

namespace custom_libraries {

class Timer_configuration {

private:
	TIM_TypeDef * TIMER; //Timer to configure
	uint16_t prescaler_value ;
	uint16_t auto_reload_value ;

private:

public:

public:
	Timer_configuration(TIM_TypeDef *TIMER, uint16_t prescaler = 0, uint16_t auto_reload_value = 0);
	~Timer_configuration();
	void set_prescaler(uint16_t prescaler_value) ;
	uint16_t get_prescaler(void) const;
	void set_auto_reload_value(uint16_t auto_reload_value) ;
	uint16_t get_auto_reload_value(void) const;
	void initialize();
};

} /* namespace custom_libraries */

#endif /* TIMERCONFIGURATION_H_ */
