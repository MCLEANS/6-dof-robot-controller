#ifndef _MG996R_H
#define _MG996R_H

#include "stm32f4xx.h"
#include "PWM.h"

#include <stdlib.h>

/**
 * 1. Clock period = 20ms (50Hz)
 * 2. When on-time is 1 ms the motor is at 0*.
 * 3. When on-time is 1.5ms the motor is at 90*.
 * 4. When on-time is 2ms the motor is at 180*.
 * 
 */

/* Configuration prescaler and Auto Reload value for 84MHz APB bus */
#define PRESCALER_APB2 84
#define ARR_VALUE_APB2 40000

/* Configuration prescaler and Auto Reload value for 42MHz APB bus*/
#define PRESCALER_APB1 84
#define ARR_VALUE_APB1 20000


#define INITIAL_POSITITON 90

namespace custom_libraries{
class MG996R : public PWM{
    private:
        uint8_t previous_angle = 0;
        uint16_t DUTY_CYCLE_MIN = 500;
        uint16_t DUTY_CYCLE_MAX = 4000;
        const uint8_t ANGLE_MIN = 0;
        const uint8_t ANGLE_MAX  = 180;
    private:
        void set_alternate_function_mode();
        void set_input_mode();
    public:
    public:
        MG996R(TIM_TypeDef *TIMER,
                channel input_channel,
                GPIO_TypeDef *PORT,
                uint8_t PIN,
                alternate_function pin_function);
        int map(long x, long in_min, long in_max, long out_min, long out_max);
        int get_duty_cycle_from_Angle(uint8_t angle);
        void move_to_angle(uint16_t angle_to);
        ~MG996R();
};

}

#endif //_MG996R_H