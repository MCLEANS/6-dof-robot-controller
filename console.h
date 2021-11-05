#ifndef CONSOLE_H
#define CONSOLE_H 

/**
 * Highlight all console commands and strings here
 */

/* FLAGS */
bool serial_handler_state = false;
bool sensor_handler_state = false;
bool motor_handler_state = false;

/* STRING LITERALS */
char STR_SOFTWARE_VERSION[] = "SOFTWARE_VERSION : ";
char STR_SENSOR_HANDLER[] = "SENSOR HANDLER : ";
char STR_SERIAL_HANDLER[] = "SERIAL HANDLER : ";
char STR_MOTOR_STATE[] = "MOTOR HANDLER : ";
char STR_SENSOR_QUEUE[] = "SENSOR QUEUE : ";
char STR_MOTOR_POSITION[] = "***MOTOR POSITION ***";
char STR_BASE_SERVO[] = "BASE SERVO : ";
char STR_SHOULDER_SERVO[] = "SHOULDER SERVO : ";
char STR_ELBOW_SERVO[] = "ELBOW SERVO : ";
char STR_WRIST_SERVO[] = "WRIST SERVO : ";

char PROJECT_NAME[] = "ROBOT CONDITION MONITORING AND CONTROL";
char SOFTWARE_VERSION[] = "SV_001.00.00";



#endif //CONSOLE_H