#include "stm32f4xx.h"
#include "clockconfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <queue.h>
#include <timers.h>
#include <string.h>

#include <MG996R.h>
#include <LIS3DH.h>
#include <USART.h>
#include <GPIO.h>
#include <ADC.h>

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

#define ZERO_VALUE "0"

/* variable to hold ADC value */
uint16_t adc_value = 0;

/* Type to hold accel and vibration sensor values */
struct Sensor_values{
  custom_libraries::Angle_values angle_values;
  int vibration_value;
};

/**
 * Servo motor objects
 */
custom_libraries::MG996R base_servo(TIM4,
                                    custom_libraries::channel1,
                                    GPIOB,
                                    6,
                                    custom_libraries::AF2);
custom_libraries::MG996R shoulder_servo(TIM4,
                                        custom_libraries::channel2,
                                        GPIOB,
                                        7,
                                        custom_libraries::AF2);
custom_libraries::MG996R wrist_servo(TIM4,
                                     custom_libraries::channel3,
                                     GPIOB,
                                     8,
                                     custom_libraries::AF2);
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
custom_libraries::USART gateway_serial(USART2, GPIOA, 3, 2, 9600);

/**
 * Create LED objects
 */
custom_libraries::_GPIO green_led(GPIOD, 12);
custom_libraries::_GPIO orange_led(GPIOD, 13);
custom_libraries::_GPIO red_led(GPIOD, 14);
custom_libraries::_GPIO blue_led(GPIOD, 15);

/**
 * Create vibration sensor object
 */
custom_libraries::_ADC vibration_sensor(ADC1, GPIOA, 1, custom_libraries::ch1, custom_libraries::FAST);

/**
 * Task handles
 */
TaskHandle_t motor_control_task;
TaskHandle_t sensor_handler_task;
TaskHandle_t gateway_serial_handler_task;

/**
 * Queue handles
 */
QueueHandle_t sensor_queue;

/**
 * Timer handles
 */
TimerHandle_t adc_timer;

/**
 * Software timer to tick every 1 ms
 */
void adc_timer_callback(TimerHandle_t xTimer){
  /* Increment the vibration sensor delay counter */
  vibration_sensor.count++;
}


/**
 * ADC interrupts handler
 */
extern "C" void ADC_IRQHandler(void)
{
  if (ADC1->SR & ADC_SR_EOC)
  {
    ADC1->SR &= ~ADC_SR_EOC;
    adc_value = ADC1->DR;
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
}

/* Convert an Integer value to a character array */
void tostring(char str[], int num)
{
  int i, rem, len = 0, n;
  n = num;
  while (n != 0)
  {
    len++; //get length for string/digits in int
    n = n / 10;
  }
  //convert and store in string
  for (i = 0; i < len; i++)
  {
    rem = num % 10;                 //last digit fetched first
    num = num / 10;                 //continue fetching rest of the digits
    str[len - (i + 1)] = rem + '0'; //start storing string with max-1 index first
  }
  str[len] = '\0'; //null to end the string[max]
}

/**
 * Serial port to handle sending data to gateway
 */
void gateway_serial_handler(void *pvParam)
{
  gateway_serial.initialize();
  /* Set up status LEDs */
  green_led.pin_mode(custom_libraries::OUTPUT);
  orange_led.pin_mode(custom_libraries::OUTPUT);
  red_led.pin_mode(custom_libraries::OUTPUT);
  blue_led.pin_mode(custom_libraries::OUTPUT);

  green_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  orange_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  red_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  blue_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  /* variable to hold values received from queue */
  Sensor_values sensor_values;
  while (1)
  {
    /* check if there is data available in queue and retreive */
    if (xQueueReceive(sensor_queue, &sensor_values, portMAX_DELAY) == pdPASS)
    {
      /* character arrays to hold values */
      char x_clockwise[10];
      char x_anticlockwise[10];
      char y_clockwise[10];
      char y_anticlockwise[10];
      char vibration[10];
      char noise[10];

      /* Accel values have been received successfully */
      if (sensor_values.angle_values.x_clockwise && sensor_values.angle_values.y_clockwise)
      {
        char payload_0[350] = "{\"xClockWise\":\"";
        if (sensor_values.angle_values.x_axis > 0)
        {
          tostring(x_clockwise, sensor_values.angle_values.x_axis);
        }
        else
        {
          strcpy(x_clockwise, ZERO_VALUE);
        }
        char payload_1[50] = "\",\"xAntiClockWise\":\"";
        strcpy(x_anticlockwise, ZERO_VALUE);
        char payload_2[50] = "\",\"y_clockwise\":\"";
        if (sensor_values.angle_values.y_axis > 0)
        {
          tostring(y_clockwise, sensor_values.angle_values.y_axis);
        }
        else
        {
          strcpy(y_clockwise, ZERO_VALUE);
        }
        char payload_3[50] = "\",\"yAntiClockWise\":\"";
        strcpy(y_anticlockwise, ZERO_VALUE);
        char payload_4[50] = "\",\"vibration\":\"";
        tostring(vibration,sensor_values.vibration_value);
        char payload_5[50] = "\",\"noise\":\"";
        strcpy(noise,ZERO_VALUE);
        char payload_6[50] = "\"}";
        strcat(payload_0, x_clockwise);
        strcat(payload_0, payload_1);
        strcat(payload_0, x_anticlockwise);
        strcat(payload_0, payload_2);
        strcat(payload_0, y_clockwise);
        strcat(payload_0, payload_3);
        strcat(payload_0, y_anticlockwise);
        strcat(payload_0, payload_4);
        strcat(payload_0, vibration);
        strcat(payload_0, payload_5);
        strcat(payload_0, noise);
        strcat(payload_0, payload_6);
        gateway_serial.println(payload_0);
        red_led.toggle();
        orange_led.toggle();
        blue_led.digital_write(0);
        green_led.digital_write(0);
      }

      else if (!sensor_values.angle_values.x_clockwise && sensor_values.angle_values.y_clockwise)
      {
        char payload_0[350] = "{\"xClockWise\":\"";
        strcpy(x_clockwise, ZERO_VALUE);
        char payload_1[50] = "\",\"xAntiClockWise\":\"";
        if (sensor_values.angle_values.x_axis > 0)
        {
          tostring(x_anticlockwise, sensor_values.angle_values.x_axis);
        }
        else
        {
          strcpy(x_anticlockwise, ZERO_VALUE);
        }
        char payload_2[50] = "\",\"yClockWise\":\"";
        if (sensor_values.angle_values.y_axis > 0)
        {
          tostring(y_clockwise, sensor_values.angle_values.y_axis);
        }
        else
        {
          strcpy(y_clockwise, ZERO_VALUE);
        }
        char payload_3[50] = "\",\"yAntiClockWise\":\"";
        strcpy(y_anticlockwise, ZERO_VALUE);
        char payload_4[50] = "\",\"vibration\":\"";
        tostring(vibration,sensor_values.vibration_value);
        char payload_5[50] = "\",\"noise\":\"";
        strcpy(noise,ZERO_VALUE);
        char payload_6[50] = "\"}";
        strcat(payload_0, x_clockwise);
        strcat(payload_0, payload_1);
        strcat(payload_0, x_anticlockwise);
        strcat(payload_0, payload_2);
        strcat(payload_0, y_clockwise);
        strcat(payload_0, payload_3);
        strcat(payload_0, y_anticlockwise);
        strcat(payload_0, payload_4);
        strcat(payload_0, vibration);
        strcat(payload_0, payload_5);
        strcat(payload_0, noise);
        strcat(payload_0, payload_6);
        gateway_serial.println(payload_0);
        orange_led.toggle();
        green_led.toggle();
        red_led.digital_write(0);
        blue_led.digital_write(0);
      }

      else if (!sensor_values.angle_values.x_clockwise && !sensor_values.angle_values.y_clockwise)
      {
        char payload_0[350] = "{\"xClockWise\":\"";
        strcpy(x_clockwise, ZERO_VALUE);
        char payload_1[50] = "\",\"xAntiClockWise\":\"";
        if (sensor_values.angle_values.x_axis > 0)
        {
          tostring(x_anticlockwise, sensor_values.angle_values.x_axis);
        }
        else
        {
          strcpy(x_anticlockwise, ZERO_VALUE);
        }
        char payload_2[50] = "\",\"yClockWise\":\"";
        strcpy(y_clockwise, ZERO_VALUE);
        char payload_3[50] = "\",\"yAntiClockWise\":\"";
        if (sensor_values.angle_values.y_axis > 0)
        {
          tostring(y_anticlockwise, sensor_values.angle_values.y_axis);
        }
        else
        {
          strcpy(y_anticlockwise, ZERO_VALUE);
        }
        char payload_4[50] = "\",\"vibration\":\"";
        tostring(vibration,sensor_values.vibration_value);
        char payload_5[50] = "\",\"noise\":\"";
        strcpy(noise,ZERO_VALUE);
        char payload_6[50] = "\"}";
        strcat(payload_0, x_clockwise);
        strcat(payload_0, payload_1);
        strcat(payload_0, x_anticlockwise);
        strcat(payload_0, payload_2);
        strcat(payload_0, y_clockwise);
        strcat(payload_0, payload_3);
        strcat(payload_0, y_anticlockwise);
        strcat(payload_0, payload_4);
        strcat(payload_0, vibration);
        strcat(payload_0, payload_5);
        strcat(payload_0, noise);
        strcat(payload_0, payload_6);
        gateway_serial.println(payload_0);
        green_led.toggle();
        blue_led.toggle();
        orange_led.digital_write(0);
        red_led.digital_write(0);
      }

      else if (sensor_values.angle_values.x_clockwise && !sensor_values.angle_values.y_clockwise)
      {
        char payload_0[350] = "{\"xClockWise\":\"";
        if (sensor_values.angle_values.x_axis > 0)
        {
          tostring(x_clockwise, sensor_values.angle_values.x_axis);
        }
        else
        {
          strcpy(x_clockwise, ZERO_VALUE);
        }
        char payload_1[50] = "\",\"xAntiClockWise\":\"";
        strcpy(x_anticlockwise, ZERO_VALUE);
        char payload_2[50] = "\",\"yClockWise\":\"";
        strcpy(y_clockwise, ZERO_VALUE);
        char payload_3[50] = "\",\"yAntiClockWise\":\"";
        if (sensor_values.angle_values.y_axis > 0)
        {
          tostring(y_anticlockwise, sensor_values.angle_values.y_axis);
        }
        else
        {
          strcpy(y_anticlockwise, ZERO_VALUE);
        }
        char payload_4[50] = "\",\"vibration\":\"";
        tostring(vibration,sensor_values.vibration_value);
        char payload_5[50] = "\",\"noise\":\"";
        strcpy(noise,ZERO_VALUE);
        char payload_6[50] = "\"}";
        strcat(payload_0, x_clockwise);
        strcat(payload_0, payload_1);
        strcat(payload_0, x_anticlockwise);
        strcat(payload_0, payload_2);
        strcat(payload_0, y_clockwise);
        strcat(payload_0, payload_3);
        strcat(payload_0, y_anticlockwise);
        strcat(payload_0, payload_4);
        strcat(payload_0, vibration);
        strcat(payload_0, payload_5);
        strcat(payload_0, noise);
        strcat(payload_0, payload_6);
        gateway_serial.println(payload_0);
        red_led.toggle();
        blue_led.toggle();
        orange_led.digital_write(0);
        green_led.digital_write(0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/**
 * Task to read Accelerometer and vibration sensor data
 */
void sensor_handler(void *pvParam)
{
  /* Initialize the motion sensor */
  accel_sensor.initialize();
  /* Structure to hold accel angle values */
  custom_libraries::Angle_values angle_values;
  /* Struct to hold sensor_data */
  Sensor_values sensor_values;
  while (1)
  {
    /* Get accelerometer angle values */
    angle_values = accel_sensor.read_angles();
    /* Store accel values */
    sensor_values.angle_values = angle_values;
    sensor_values.vibration_value = adc_value;
    /* Check if item was sucessfully added to queue */
    if (xQueueSend(sensor_queue, (void *)&sensor_values, (TickType_t)0 == pdPASS))
    {
      /* Item added to queue succesfully */
    }
    /* Block the task */
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/**
 * Task to control robot motors
 */
void motor_controller(void *pvParam)
{
  /* base motor rest position */
  base_servo.move_to_angle(90);
  while (1)
  {
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

int main(void)
{
  /* Initialize system clock */
  system_clock.initialize();
  /* Initialize vibration sensor */
  vibration_sensor.initialize();
  /**
   * Set-up vibration sensor ADC interrupts
   * (When using FreeRTOS interrupt priority should not below 0x05)
   */
  NVIC_SetPriority(ADC_IRQn, 0x06);
  NVIC_EnableIRQ(ADC_IRQn);
  /**
   * Create queue to hold accelerometer angle values
   */
  sensor_queue = xQueueCreate(10, sizeof(Sensor_values));
  /**
   * Create adc timer
   */
  adc_timer = xTimerCreate("Timer to be used by ADC",
                            pdMS_TO_TICKS(1),
                            pdTRUE,
                            (void*)0,
                            adc_timer_callback);
  if(adc_timer != NULL){
    xTimerStart(adc_timer,0);
  }

  /* create system tasks */
  xTaskCreate(motor_controller,
              "Motor Control Task",
              100,
              NULL,
              2,
              &motor_control_task);
  xTaskCreate(sensor_handler,
              "Task to handle reading data from accelerometer and vibration sensor",
              200,
              NULL,
              2,
              &sensor_handler_task);
  xTaskCreate(gateway_serial_handler,
              "Task to handle sending data to the gateway",
              1000,
              NULL,
              2,
              &gateway_serial_handler_task);

  /* Start system scheduler */
  vTaskStartScheduler();

  while (1)
  {
  }
}
