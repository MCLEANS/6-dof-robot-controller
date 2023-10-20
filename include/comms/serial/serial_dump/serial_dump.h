//
// Created by mcleans on 9/30/23.
//

#ifndef ROBOT_CONTROLLER_SERIAL_DUMP_H
#define ROBOT_CONTROLLER_SERIAL_DUMP_H

#include "USART.h"

class SerialDump {
private:
    /**
     * Serial Communication UART interface
     */
    custom_libraries::USART _gateway_serial;

public:
    /**
     * Default constructor
     */
    SerialDump();

    /**
     * Initialize the serial dumper
     * @return 0 on success, -1 on failure
     */
    int init();

    /**
     * Dump data to the serial interface
     * @param buffer payload to dump
     * @param len size of the payload
     * @return number of bytes dumped
     */
    int dump(char* buffer, int len);

    /**
     * @return default instance of the serial dumper
     */
    static SerialDump* get_instance();

};

#endif //ROBOT_CONTROLLER_SERIAL_DUMP_H
