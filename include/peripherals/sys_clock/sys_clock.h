/*
 * sys_clock.h
 *
 *  Created on: Mar 30, 2020
 *      Author: MCLEANS
 */

#ifndef SYS_CLOCK_H
#define SYS_CLOCK_H

#include "stm32f4xx.h"

namespace custom_libraries {

class sys_clock {
public:
	sys_clock();
	~sys_clock();
	void initialize();
};

} /* namespace custom_libraries */

#endif /* SYS_CLOCK_H */
