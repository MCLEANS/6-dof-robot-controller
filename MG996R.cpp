#include "MG996R.h"

namespace custom_libraries{

MG996R::MG996R(TIM_TypeDef *TIMER,
                channel input_channel,
                GPIO_TypeDef *PORT,
                uint8_t PIN,
                alternate_function pin_function):PWM(TIMER,
                                                input_channel,
                                                PORT,
                                                PIN,
                                                pin_function,
                                                PRESCALER_APB1,
                                                ARR_VALUE_APB1){
    /* Initialize PWM */
    begin();

    /**
     * Check timer used and set prescaler and auto-reload value accordingly
     */
    if((TIMER == TIM1) || (TIMER == TIM10)){
        set_prescaler(PRESCALER_APB2);
        set_auto_reload_value(ARR_VALUE_APB2);
        /* Configure maximum and minimum duty cycle */
        DUTY_CYCLE_MIN = 1000;
        DUTY_CYCLE_MAX = 4000;
    }
    else{
        set_prescaler(PRESCALER_APB1);
        set_auto_reload_value(ARR_VALUE_APB1);
        /* Configure maximum and minimum duty cycle */
        DUTY_CYCLE_MIN = 500;
        DUTY_CYCLE_MAX = 3000;
    }

    /* Set initial position to 90 degrees */
    set_duty_cycle(get_duty_cycle_from_Angle(INITIAL_POSITITON));
    this->previous_angle = INITIAL_POSITITON;

 }

/**
 * custom map function
 */
int MG996R::map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Function to generate duty cycle from angle
 */
int MG996R::get_duty_cycle_from_Angle(uint8_t angle){
  int duty_cycle = map(angle,ANGLE_MIN,ANGLE_MAX,DUTY_CYCLE_MIN,DUTY_CYCLE_MAX);
  return duty_cycle;
}

/**
 * Function to move the servo onto a particular angle
 */
void MG996R::move_to_angle(uint16_t angle_to){
  
  /* calculate differential angle */
  int differential_angle = this->previous_angle - angle_to;
  if(differential_angle < 0){
    /* Set PWM Pin to alternate output mode */
    set_alternate_function_mode();
    
    for(int i = 0 ; i < abs(differential_angle); i++){
        set_duty_cycle(get_duty_cycle_from_Angle(previous_angle+i));
        //Put a small pseudo delay
        for(volatile int i = 0; i < 300000; i++){}
    }

    previous_angle = angle_to;
  }

  if(differential_angle > 0){
    /* Set PWM Pin to alternate output mode */
    set_alternate_function_mode();

    for(int i = 0; i < abs(differential_angle); i++){
        set_duty_cycle(get_duty_cycle_from_Angle(previous_angle-i));
        //Put a small pseudo delay
        for(volatile int i = 0; i < 350000; i++){}
    }

    previous_angle = angle_to;
  }

  if(differential_angle == 0){
    //Do nothing current PWM Signal perists
  }
}

/**
 * Confiure pin to alternate function to support PWM
 */
void MG996R::set_alternate_function_mode(){
  PORT->MODER &= ~(1 << (PIN*2));
	PORT->MODER |= (1 << ((PIN*2)+1));
}

/**
 * Configure pin to input mode which is the default reset state
 */
void MG996R::set_input_mode(){
  PORT->MODER &= ~(1 << (PIN*2));
	PORT->MODER &= ~(1 << ((PIN*2)+1));
}

MG996R::~MG996R(){

 }

}

