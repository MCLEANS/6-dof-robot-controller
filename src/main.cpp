#include "common.h"

/* variable to hold ADC value */
uint16_t raw_vibration_value = 0;

/* Type to hold accel and vibration sensor values */
struct Sensor_values{
  custom_libraries::Angle_values angle_values;
  int vibration_value;
};

/* SYSTEM TASK HANDLES */
TaskHandle_t motor_control_task;
TaskHandle_t sensor_handler_task;
TaskHandle_t gateway_serial_handler_task;
TaskHandle_t debug_console_handler_task;

/* QUEUE HANDLES */
QueueHandle_t sensor_queue;

/* TIMER HANDLES */
TimerHandle_t flag_reset_timer;

/* SEMAPHORE HANDLES */
SemaphoreHandle_t debug_handler;

/* ACCELEROMETER INTERRUPT HANDLER */
extern "C" void ADC_IRQHandler(void){
  if (ADC1->SR & ADC_SR_EOC){
    ADC1->SR &= ~ADC_SR_EOC;
    raw_vibration_value = ADC1->DR;
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
}

/**
 * Debug button interrupt handler
 */
//TO-DO : This is not the exact interrupt pin, to be modified accrodingly
extern "C" void EXTI0_IRQHandler(void){
  static BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  if(EXTI->PR & EXTI_PR_PR0){
    /* Do something here */
    EXTI->PR |= EXTI_PR_PR0;
    xSemaphoreGiveFromISR(debug_handler, &xHigherPriorityTaskWoken);
	}
  if(xHigherPriorityTaskWoken == pdTRUE){
    /* Yield here */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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

/* Debug serial console */
void debug_console_handler(void *pvParam){
  /* Initialize debug button */
  debug_button.initialize();
  /* Enable interrupts for debug button */
  NVIC_SetPriority(EXTI0_IRQn,0x07);
  NVIC_EnableIRQ(EXTI0_IRQn);

  char base_servo_angle[4];
  char shoulder_servo_angle[4];
  char elbow_servo_angle[4];
  char wrist_servo_angle[4];

  /* Initialize the degug console */
  debug_console.initialize();
  while(1){

    if( xSemaphoreTake( debug_handler, portMAX_DELAY ) == pdTRUE ){
      /* Title */
      DEBUG_LN(STR_lOGO_1);
      DEBUG_LN(STR_lOGO_2);
      DEBUG_LN(STR_lOGO_3);
      DEBUG_LN(STR_lOGO_4);
      DEBUG_LN(STR_lOGO_5);
      DEBUG_LN(STR_lOGO_6);
      DEBUG_LN(STR_lOGO_7);
      DEBUG_LN(STR_EMPTY);
      DEBUG_LN(STR_PARTITION);
      DEBUG_LN(STR_EMPTY);
      DEBUG_LN(PROJECT_NAME);
      DEBUG(STR_SOFTWARE_VERSION);
      DEBUG_LN(SOFTWARE_VERSION);

      /* Flags */
      DEBUG_LN(STR_EMPTY);
      DEBUG(STR_SENSOR_HANDLER);
      if(sensor_handler_state) DEBUG_LN(STR_OK);
      else DEBUG_LN(STR_ERROR);
      DEBUG(STR_SERIAL_HANDLER);
      if(serial_handler_state) DEBUG_LN(STR_OK);
      else DEBUG_LN(STR_ERROR);
      DEBUG(STR_MOTOR_HANDLER);
      if(motor_handler_state) DEBUG_LN(STR_OK);
      else DEBUG_LN(STR_ERROR);
      DEBUG(STR_SENSOR_QUEUE);
      if (sensor_queue_state) DEBUG_LN(STR_OK);
      else DEBUG_LN(STR_ERROR);

      /* Motor Position */
      //convert integer angle to strings
      tostring(base_servo_angle,base_servo.get_current_angle());
      tostring(shoulder_servo_angle,shoulder_servo.get_current_angle());
      tostring(elbow_servo_angle,elbow_servo.get_current_angle());
      tostring(wrist_servo_angle,wrist_servo.get_current_angle());

      DEBUG_LN(STR_EMPTY);
      DEBUG_LN(STR_MOTOR_POSITION);
      DEBUG(STR_BASE_SERVO);
      DEBUG_LN(base_servo_angle);
      DEBUG(STR_SHOULDER_SERVO);
      DEBUG_LN(shoulder_servo_angle);
      DEBUG(STR_ELBOW_SERVO);
      DEBUG_LN(elbow_servo_angle);
      DEBUG(STR_WRIST_SERVO);
      DEBUG_LN(wrist_servo_angle);
      DEBUG_LN(STR_EMPTY);
      DEBUG_LN(STR_PARTITION);
    }
  }
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
  gateway_handler_led.pin_mode(custom_libraries::OUTPUT);
  sensor_queue_receive_led.pin_mode(custom_libraries::OUTPUT);

  green_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  orange_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  red_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  blue_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  gateway_handler_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  sensor_queue_receive_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);

  /* variable to hold values received from queue */
  Sensor_values sensor_values;
  while (1)
  {
    /* check if there is data available in queue and retreive */
    if (xQueueReceive(sensor_queue, &sensor_values, portMAX_DELAY) == pdPASS)
    {
      sensor_queue_receive_led.toggle();
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
        char payload_0[1024] = "{\"xClockWise\":\"";
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
        if (sensor_values.vibration_value > 0){
          tostring(vibration,sensor_values.vibration_value);
        } 
        else{
          strcpy(vibration,ZERO_VALUE);
        }
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
        char payload_0[1024] = "{\"xClockWise\":\"";
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
        if (sensor_values.vibration_value > 0){
          tostring(vibration,sensor_values.vibration_value);
        } 
        else{
          strcpy(vibration,ZERO_VALUE);
        }
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
        char payload_0[1024] = "{\"xClockWise\":\"";
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
        if (sensor_values.vibration_value > 0){
          tostring(vibration,sensor_values.vibration_value);
        } 
        else{
          strcpy(vibration,ZERO_VALUE);
        }
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
        char payload_0[1024] = "{\"xClockWise\":\"";
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
        if (sensor_values.vibration_value > 0){
          tostring(vibration,sensor_values.vibration_value);
        } 
        else{
          strcpy(vibration,ZERO_VALUE);
        }
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
    gateway_handler_led.toggle();
    serial_handler_state = true;
    vTaskDelay(pdMS_TO_TICKS(SERIAL_HANDLER_BLOCK_TIME));
  }
}
/**
 * Task to read Accelerometer and vibration sensor data
 */
void sensor_handler(void *pvParam)
{
  sensor_handler_led.pin_mode(custom_libraries::OUTPUT);
  sensor_handler_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  sensor_queue_send_led.pin_mode(custom_libraries::OUTPUT);
  sensor_queue_send_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
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
    /* Check if item was sucessfully added to queue */
    if (xQueueSend(sensor_queue, (void *)&sensor_values, (TickType_t)0 == pdPASS))
    {
      /* Item added to queue succesfully */
      sensor_queue_send_led.toggle();
      sensor_queue_state = true;
    }
    else{
      /* Item could not be added to the queue, handle error */
      sensor_queue_state = false;
    }
    sensor_handler_led.toggle();
    sensor_handler_state = true;
    /* Block the task */
    vTaskDelay(pdMS_TO_TICKS(SENSOR_HANDLER_BLOCK_TIME));
  }
}

/**
 * Task to control robot motors
 */
void motor_controller(void *pvParam)
{
  /**
   * NOTE : The motor control led is used to show servo motor movements
   */
  motor_control_led.pin_mode(custom_libraries::OUTPUT);
  motor_control_led.output_settings(custom_libraries::PUSH_PULL, custom_libraries::VERY_HIGH);
  /* base motor rest position */
  while (1)
  {
    
    motor_handler_state = true;
    wrist_servo.move_to_angle(20);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* Timer to reset flags */
void flag_reset(TimerHandle_t xTimer){
  /* Reset flags */
    serial_handler_state = false;
    sensor_handler_state = false;
    motor_handler_state = false;
    sensor_queue_state = false;
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

  /* Create flag reset timer */
  flag_reset_timer = xTimerCreate("Timer to reset flags",
                                    pdMS_TO_TICKS(2000),
                                    pdTRUE,
                                    (void*)0,
                                    flag_reset);

  /**
   * Create queue to hold accelerometer angle values
   */
  sensor_queue = xQueueCreate(10, sizeof(Sensor_values));

  /**
   * Create semaphore to handle debug console
   */
  debug_handler = xSemaphoreCreateBinary();
  if(debug_handler == NULL){
    /* There wasn't enough space to create semaphore */
    /* Error handling here */
  }
  else{
    /* Semaphore created sucessfully */
    xSemaphoreGive(debug_handler);
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
              1,
              &sensor_handler_task);

  xTaskCreate(gateway_serial_handler,
              "Task to handle sending data to the gateway",
              1000,
              NULL,
              1,
              &gateway_serial_handler_task);

  xTaskCreate(debug_console_handler,
              "Task to handle serial debug console",
              1000,
              NULL,
              1,
              &debug_console_handler_task);

  /* Start system scheduler */
  vTaskStartScheduler();

  while (1)
  {
    
  }
}


 