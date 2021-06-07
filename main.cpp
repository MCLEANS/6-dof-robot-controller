#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <MG996R.h>

/**
 *  To Do : Read Angle data from accelerometer
 *          Create task to perform data reading.
 *          Create Serial output to flash out data
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

custom_libraries::clock_config system_clock;

/**
 * Task handles
 */
TaskHandle_t motor_control_task;

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