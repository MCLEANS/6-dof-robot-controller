#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <MG996R.h>
#include <LIS3DH.h>
#include <USART.h>

/**
 * To Do
 * Add Queue to hold accel data and read data from the queue in the gateway serial handler
 */

/**
 * LIS3DH sensor Pin Mapping
 */
#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6
#define CS_PORT GPIOE
#define CS_PIN 3

/**
 * Servo motor objects
 */
custom_libraries::MG996R base_servo(TIM4,
                                    custom_libraries::channel1,
                                    GPIOB,
                                    6,
                                    custom_libraries::AF2
                                    );
custom_libraries::MG996R shoulder_servo(TIM4,
                                        custom_libraries::channel2,
                                        GPIOB,
                                        7,
                                        custom_libraries::AF2
                                        );
custom_libraries::MG996R wrist_servo(TIM4,
                                    custom_libraries::channel3,
                                    GPIOB,
                                    8,
                                    custom_libraries::AF2
                                    );
/**
 * System clock configuration
 */
custom_libraries::clock_config system_clock;

/**
 * Accelerometer object
 */
custom_libraries::LIS3DH accel_sensor(SPI1,
                                        GPIOA,
                                        SCK_PIN,
                                        MOSI_PIN,
                                        MISO_PIN,
                                        CS_PORT,
                                        CS_PIN);

/**
 * Serial port to send data to gateway
 */
custom_libraries::USART gateway_serial(USART1,GPIOA,10,9,115200);

/**
 * Task handles
 */
TaskHandle_t motor_control_task;
TaskHandle_t accelerometer_handler_task;
TaskHandle_t gateway_serial_handler_task;

/**
 * Serial port to handle sending data to gateway
 */
void gateway_serial_handler(void* pvParam){
  /* Initialize serial port */
  gateway_serial.initialize();
  while(1){

  }
}

/**
 * Task to read Accelerometer data
 */
void accelerometer_handler(void* pvParam){
  /* Initialize the motion sensor */
  accel_sensor.initialize();
  /* Structure to hold accel angle values */
  custom_libraries::Angle_values angle_values;
  while(1){
    angle_values = accel_sensor.read_angles();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/**
 * Task to control robot motors
 */
void motor_controller(void* pvParam){
  /* base motor rest position */
  base_servo.move_to_angle(90);
  while(1){
    base_servo.move_to_angle(20);
    vTaskDelay(pdMS_TO_TICKS(1000));
    shoulder_servo.move_to_angle(60);
    vTaskDelay(pdMS_TO_TICKS(300));
    shoulder_servo.move_to_angle(100);
    vTaskDelay(pdMS_TO_TICKS(300));
    base_servo.move_to_angle(120);
    vTaskDelay(pdMS_TO_TICKS(1000));
    shoulder_servo.move_to_angle(60);
    vTaskDelay(pdMS_TO_TICKS(300));
    shoulder_servo.move_to_angle(100); 
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

int main(void) {
  system_clock.initialize();

  /* create system tasks */
  xTaskCreate(motor_controller,
              "Motor Control Task",
              100,
              NULL,
              1,
              &motor_control_task);
  xTaskCreate(accelerometer_handler,
              "Task to handle reading data from accelerometer",
              100,
              NULL,
              1,
              &accelerometer_handler_task);
  xTaskCreate(gateway_serial_handler,
              "Task to handle sending data to the gateway",
              100,
              NULL,
              1,
              &gateway_serial_handler_task);

  /* Start system scheduler */
  vTaskStartScheduler();

  while(1){}
}