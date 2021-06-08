#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <MG996R.h>
#include <LIS3DH.h>

/**
 * LIS3DH sensor Pin Mapping
 */
#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6
#define CS_PORT GPIOE
#define CS_PIN 3

/**
 *  To Do : Read Angle data from accelerometer
 *          Create task to perform data reading.
 *          Create Serial output to flash out data
 */


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
custom_libraries::LIS3DH motion_sensor(SPI1,
                                        GPIOA,
                                        SCK_PIN,
                                        MOSI_PIN,
                                        MISO_PIN,
                                        CS_PORT,
                                        CS_PIN);

/**
 * Task handles
 */
TaskHandle_t motor_control_task;
TaskHandle_t accelerometer_task;

/**
 * Task to control robot motors
 */
void motor_control(void* pvParam){
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
  xTaskCreate(motor_control,
              "Motor Control Task",
              100,
              NULL,
              1,
              &motor_control_task);

  /* Start system scheduler */
  vTaskStartScheduler();

  while(1){}
}