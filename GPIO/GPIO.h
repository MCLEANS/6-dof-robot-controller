/*
 * GPIO.h
 *
 *  Created on: May 4, 2020
 *      Author: MCLEANS
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4xx.h"

namespace custom_libraries {

enum Mode{
	INPUT,
	OUTPUT
};

enum OUTPUT_Type{
	PUSH_PULL,
	OPEN_DRAIN
};

enum OUTPUT_Speed{
	LOW,
	MEDUIM,
	HIGH,
	VERY_HIGH
};

enum State{
	FLOATING,
	PULL_UP,
	PULL_DOWN
};


class _GPIO {
private:
	GPIO_TypeDef *GPIO;
	uint8_t PIN;
	Mode mode;
	OUTPUT_Type type;
	OUTPUT_Speed speed;
	State state;
private:
public:
public:
	_GPIO(GPIO_TypeDef *GPIO,uint8_t PIN);
	void pin_mode(Mode mode);
	void output_settings(OUTPUT_Type type, OUTPUT_Speed speed);
	void input_state(State state);
	void digital_write(bool value);
	bool digital_read(void) const;
	void toggle();

	~_GPIO();
};

} /* namespace custom_libraries */

#endif /* GPIO_H_ */
