#include "common.h"
#include "serial_comms.h"
#include "accel_event.h"

/* variable to hold ADC value */
uint16_t raw_vibration_value = 0;

/* Type to hold accel and vibration sensor values */
struct Sensor_values{
  custom_libraries::Angle_values angle_values;
  int vibration_value;
};

/* SYSTEM TASK HANDLES */
TaskHandle_t sensor_handler_task;

/* ACCELEROMETER INTERRUPT HANDLER */
extern "C" void ADC_IRQHandler(void){
  if (ADC1->SR & ADC_SR_EOC){
    ADC1->SR &= ~ADC_SR_EOC;
    raw_vibration_value = ADC1->DR;
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
}

/**
 * Task to read Accelerometer and vibration sensor data
 */
void sensor_handler(void *pvParam)
{

  /* Initialize the motion sensor */
  accel_sensor.initialize();
  /* Struct to hold sensor_data */
  Sensor_values sensor_values;
  
  while (1)
  {
    /* Store accel values */
    sensor_values.angle_values = accel_sensor.read_angles();
    /* Obtain vibration sensor ADC value */
    sensor_values.vibration_value = raw_vibration_value;

    /** Emit values */
    s_accel_ev_t _ev;
    _ev.x_cws = sensor_values.angle_values.x_axis;
    SerialAccelEvent::get_instance()->emit(&_ev);

    /* Block the task */
    vTaskDelay(pdMS_TO_TICKS(SENSOR_HANDLER_BLOCK_TIME));
  }
}

int main(void)
{
  /* Initialize system clock */
  system_clock.initialize();

  /**
   * Initialize vibration sensor 
   **/
  vibration_sensor.initialize();

  /**
   * Set-up vibration sensor ADC interrupts
   * (When using FreeRTOS interrupt priority should not below 0x05)
   */
  NVIC_SetPriority(ADC_IRQn, 0x06);
  NVIC_EnableIRQ(ADC_IRQn);

  /**
   * Initialize Serial comms
   * 
   */
  SerialComms::get_instance()->init();

  xTaskCreate(sensor_handler,
              "Task to handle reading data from accelerometer and vibration sensor",
              200,
              NULL,
              1,
              &sensor_handler_task);

  xTaskCreate(SerialComms::run,
              "Task to handle sending data to the gateway",
              1000,
              NULL,
              1,
              &SerialComms::get_instance()->taskHandle);

  /* Start system scheduler */
  vTaskStartScheduler();

  while (1)
  {
    
  }
}


 