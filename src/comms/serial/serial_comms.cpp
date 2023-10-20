//
// Created by mcleans on 10/1/23.
//

#include "serial_comms.h"
#include "serial_dump.h"
#include "serial_events.h"

void SerialComms::init()
{
    SerialDump::get_instance()->init();
}

void SerialComms::run(void* pv_param)
{

    SerialCommsEvents* _event; 

    while(1){

        if(xQueueReceive(*SerialCommsEventsQueue::get_default_instance(),
                            (void*) _event,
                            (TickType_t)0) == pdPASS){
                
                    _event->post();      
   
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

SerialComms* SerialComms::get_instance()
{
    static SerialComms serial_comms;
    return &serial_comms;

}