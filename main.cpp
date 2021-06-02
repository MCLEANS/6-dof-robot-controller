#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

custom_libraries::clock_config system_clock;

int main(void) {
  system_clock.initialize();
  
  while(1){}
}