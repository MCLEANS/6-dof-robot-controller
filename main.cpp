#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <string>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>

#include <queue.h>

#include <MG996R.h>
#include <LIS3DH.h>
#include <USART.h>
#include <GPIO.h>


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

#define ZERO_VALUE 0

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
custom_libraries::USART gateway_serial(USART2,GPIOA,3,2,9600);

/**
 * Create LED objects
 */
custom_libraries::_GPIO green_led(GPIOD,12);
custom_libraries::_GPIO orange_led(GPIOD,13);
custom_libraries::_GPIO red_led(GPIOD,14);
custom_libraries::_GPIO blue_led(GPIOD,15);

/**
 * Task handles
 */
TaskHandle_t motor_control_task;
TaskHandle_t accelerometer_handler_task;
TaskHandle_t gateway_serial_handler_task;

/**
 * Queue handles
 */
QueueHandle_t accel_queue;

/* Convert an Integer value to a character array */
void tostring(char str[], int num){
  int i, rem, len =0, n;
  n=num;
  while(n!=0){
    len++; //get length for string/digits in int
    n=n/10;
  }
  //convert and store in string
  for(i=0;i<len;i++) {
    rem=num%10; //last digit fetched first
    num=num/10; //continue fetching rest of the digits
    str[len-(i+1)]=rem + '0'; //start storing string with max-1 index first
  }
  str[len]='\0'; //null to end the string[max]
}

/**
 * Serial port to handle sending data to gateway
 */
void gateway_serial_handler(void* pvParam){
  /* Set up status LEDs */
  green_led.pin_mode(custom_libraries::OUTPUT);
  orange_led.pin_mode(custom_libraries::OUTPUT);
  red_led.pin_mode(custom_libraries::OUTPUT);
  blue_led.pin_mode(custom_libraries::OUTPUT);

  green_led.output_settings(custom_libraries::PUSH_PULL,custom_libraries::VERY_HIGH);
  orange_led.output_settings(custom_libraries::PUSH_PULL,custom_libraries::VERY_HIGH);
  red_led.output_settings(custom_libraries::PUSH_PULL,custom_libraries::VERY_HIGH);
  blue_led.output_settings(custom_libraries::PUSH_PULL,custom_libraries::VERY_HIGH);
  /* variable to hold values received from queue */
  custom_libraries::Angle_values angle_values;
  char data[10];
  while(1){
    /* check if there is data available in queue and retreive */
    if(xQueueReceive(accel_queue, &angle_values, (TickType_t)0) == pdPASS){
      /* Accel values have been received successfully */
      tostring(data,angle_values.x_axis);
      gateway_serial.println(data);
      if(angle_values.x_clockwise){
        green_led.toggle();
        red_led.digital_write(0);
      }
      else if(!angle_values.x_clockwise){
        red_led.toggle();
        green_led.digital_write(0);
      }
      if(angle_values.y_clockwise){
        blue_led.toggle();
        orange_led.digital_write(0);
      }
      else if(!angle_values.y_clockwise){
        orange_led.toggle();
        blue_led.digital_write(0);
      }
      
    }
    vTaskDelay(pdMS_TO_TICKS(50));
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
    /* Get accelerometer angle values */
    angle_values = accel_sensor.read_angles();
    /* Check if item was sucessfully added to queue */
    if(xQueueSend(accel_queue, (void*)&angle_values,(TickType_t)0 == pdPASS)){
      /* Item added to queue succesfully */
    }
    /* Block the task */
    vTaskDelay(pdMS_TO_TICKS(20));
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
  gateway_serial.initialize();

  /**
   * Create queue to hold accelerometer angle values
   */
  accel_queue = xQueueCreate(10,sizeof(custom_libraries::Angle_values));

  /* create system tasks */
  xTaskCreate(motor_controller,
              "Motor Control Task",
              100,
              NULL,
              1,
              &motor_control_task);
  xTaskCreate(accelerometer_handler,
              "Task to handle reading data from accelerometer",
              200,
              NULL,
              3,
              &accelerometer_handler_task);
  xTaskCreate(gateway_serial_handler,
              "Task to handle sending data to the gateway",
              200,
              NULL,
              2,
              &gateway_serial_handler_task);

  /* Start system scheduler */
  vTaskStartScheduler();

  while(1){}
}