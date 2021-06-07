#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <MG996R.h>

custom_libraries::MG996R base_servo(TIM4,
                                    custom_libraries::channel1,
                                    GPIOB,
                                    6,
                                    custom_libraries::AF2
                                    );

/**
 *  To Do : Read Angle data from accelerometer
 *          Create task to perform data reading.
 *          Create Serial output to flash out data
 */


custom_libraries::clock_config system_clock;

int main(void) {
  system_clock.initialize();

  while(1){}
}