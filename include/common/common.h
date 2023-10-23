#ifndef _COMMON_H
#define _COMMON_H

#include "stm32f4xx.h"

/* RTOS HEADER FILES */
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <string.h>

/* SENSOR / ACTUATOR HEADER FILES */
#include <MG996R.h>
#include <LIS3DH.h>

/* PERIPHERAL LIBRARY HEADER FILES */
#include <USART.h>
#include <GPIO.h>
#include <ADC.h>
#include <sys_clock.h>

#include "console.h"


/* RTOS TASK BLOCK DURATIONS IN MS*/
#define SERIAL_HANDLER_BLOCK_TIME 100
#define SENSOR_HANDLER_BLOCK_TIME 200

/* LIS3DH PIN MAPPING */
#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6
#define CS_PORT GPIOE
#define CS_PIN 3

/* SERVO MOTOR INSTANCES */
custom_libraries::MG996R base_servo(TIM4, custom_libraries::channel1, GPIOB, 6, custom_libraries::AF2);
custom_libraries::MG996R shoulder_servo(TIM4, custom_libraries::channel2, GPIOB, 7, custom_libraries::AF2);
custom_libraries::MG996R elbow_servo(TIM3, custom_libraries::channel3, GPIOB, 0, custom_libraries::AF2);     
custom_libraries::MG996R wrist_servo(TIM3, custom_libraries::channel4, GPIOB, 1,custom_libraries::AF2);                                  

/********************************************************************/
custom_libraries::MG996R wrist_servo1(TIM4, custom_libraries::channel3, GPIOB, 10, custom_libraries::AF2);
custom_libraries::MG996R base_servo1(TIM4,  custom_libraries::channel4, GPIOB,  9,  custom_libraries::AF2);

/********************************************************************/

/* SYSTEM CLOCK CONFIGURATION */
custom_libraries::sys_clock system_clock;

/* ACCELEROMETER INSTANCE */
custom_libraries::LIS3DH accel_sensor(SPI1, GPIOA, SCK_PIN, MOSI_PIN, MISO_PIN, CS_PORT, CS_PIN);

/* VIBRATION SENSOR INSTANCE*/
custom_libraries::_ADC vibration_sensor(ADC1, GPIOA, 4, custom_libraries::ch4, custom_libraries::SLOW);


#endif