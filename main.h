#ifndef _MAIN_H
#define _MAIN_H

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
#include <EXTI.h>
#include <clockconfig.h>

#include "console.h"


/* RTOS TASK BLOCK DURATIONS IN MS*/
#define SERIAL_HANDLER_BLOCK_TIME 100
#define SENSOR_HANDLER_BLOCK_TIME 50

/* LIS3DH PIN MAPPING */
#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6
#define CS_PORT GPIOE
#define CS_PIN 3

#define ZERO_VALUE "0"

/* DEBUG BUTTON PIN MAPPING */
#define DEBUG_BUTTON_PORT GPIOE
#define DEBUG_BUTTON_PIN 0

/* DEBUG PORT PIN MAPPING */
#define DEBUG_PORT GPIOA
#define DEBUG_PORT_RX 3
#define DEBUG_PORT_TX 2

/* SERVO MOTOR INSTANCES */
custom_libraries::MG996R base_servo(TIM4, custom_libraries::channel1, GPIOB, 6, custom_libraries::AF2);
custom_libraries::MG996R shoulder_servo(TIM4, custom_libraries::channel2, GPIOB, 9, custom_libraries::AF2);
custom_libraries::MG996R elbow_servo(TIM3, custom_libraries::channel3, GPIOB, 0, custom_libraries::AF2);     
custom_libraries::MG996R wrist_servo(TIM3, custom_libraries::channel4, GPIOB, 1,custom_libraries::AF2);                                  

/********************************************************************/
custom_libraries::MG996R wrist_servo1(TIM4, custom_libraries::channel3, GPIOB, 10, custom_libraries::AF2);
custom_libraries::MG996R base_servo1(TIM4,  custom_libraries::channel4, GPIOB,  7,  custom_libraries::AF2);

/********************************************************************/

/* EXTERNAL INTERRUPTS */
custom_libraries::edge response_edge = custom_libraries::FALLING;
custom_libraries::_EXTI debug_button(DEBUG_BUTTON_PORT,DEBUG_BUTTON_PIN,response_edge);

/* SYSTEM CLOCK CONFIGURATION */
custom_libraries::clock_config system_clock;

/* ACCELEROMETER INSTANCE */
custom_libraries::LIS3DH accel_sensor(SPI1, GPIOA, SCK_PIN, MOSI_PIN, MISO_PIN, CS_PORT, CS_PIN);

/* GATEWAY SERIAL PORT */
custom_libraries::USART gateway_serial(USART1, GPIOB, 7, 6, 9600);

/* DEBUG CONSOLE */
custom_libraries::USART debug_console(USART2, DEBUG_PORT, DEBUG_PORT_RX, DEBUG_PORT_TX, 9600);
#define DEBUG(message) (debug_console.print(message)) 
#define DEBUG_LN(message) (debug_console.println(message)) 

/* SYSTEM LEDS PINMAP*/
custom_libraries::_GPIO green_led(GPIOD, 12);
custom_libraries::_GPIO orange_led(GPIOD, 13);
custom_libraries::_GPIO red_led(GPIOD, 14);
custom_libraries::_GPIO blue_led(GPIOD, 15);

custom_libraries::_GPIO motor_control_led(GPIOD, 0);
custom_libraries::_GPIO sensor_handler_led(GPIOD, 1);
custom_libraries::_GPIO gateway_handler_led(GPIOD, 2);
custom_libraries::_GPIO sensor_queue_send_led(GPIOD, 4);
custom_libraries::_GPIO sensor_queue_receive_led(GPIOD, 6);

/* VIBRATION SENSOR INSTANCE*/
custom_libraries::_ADC vibration_sensor(ADC1, GPIOA, 4, custom_libraries::ch4, custom_libraries::SLOW);


#endif