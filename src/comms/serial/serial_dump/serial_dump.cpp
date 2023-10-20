//
// Created by mcleans on 9/30/23.
//

#include "serial_dump.h"
#include "pinout_config.h"

SerialDump::SerialDump():_gateway_serial(ROBOT_CONTROLLER_PINOUT_GATEWAY_UART,
                                            ROBOT_CONTROLLER_PINOUT_GATEWAY_UART_PORT,
                                            ROBOT_CONTROLLER_PINOUT_GATEWAY_UART_RX,
                                            ROBOT_CONTROLLER_PINOUT_GATEWAY_UART_TX,
                                            ROBOT_CONTROLLER_PINOUT_GATEWAY_UART_BAUD)
{

}


int SerialDump::init()
{
    _gateway_serial.initialize();
    return 0;
}

int SerialDump::dump(char* buffer, int len)
{
    _gateway_serial.println(buffer);
    return len;
}

SerialDump* SerialDump::get_instance()
{
    static SerialDump serial_dump;
    return &serial_dump;
}