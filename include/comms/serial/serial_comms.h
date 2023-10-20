//
// Created by mcleans on 10/1/23.
//

#ifndef ROBOT_CONTROLLER_SERIAL_COMMS_H
#define ROBOT_CONTROLLER_SERIAL_COMMS_H

#include "FreeRTOS.h"
#include "task.h"

class SerialComms {
public:
    /**
     * Initialize Serial comms
     */
    void init();

    /**
     * Run Serial comms
     */
    static void run(void* pv_param);

    /**
     * @brief Get the instance object
     * 
     * @return SerialComms* default instance of serial comms
     */
    static SerialComms* get_instance();

    /**
     * @brief Serial Comms FreeRTOS task handle
     * 
     */
    TaskHandle_t taskHandle;

};

#endif //ROBOT_CONTROLLER_SERIAL_COMMS_H
