//
// Created by mcleans on 9/30/23.
//
#include "accel_event.h"
#include "serial_dump.h"
#include <string.h>

int SerialAccelEvent::emit(void* ev)
{
    _ev = static_cast<s_accel_ev_t*>(ev);

    if (xQueueSend(*SerialCommsEventsQueue::get_default_instance(),
               (void *)this,
               (TickType_t) 0) != pdPASS){
        return -1;
    }
    return 0;
}

void SerialAccelEvent::post()
{
    char test_buffer[] = "{\"some json key\":\"some json value\"}";
    SerialDump::get_instance()->dump(test_buffer, strlen(test_buffer));
}

SerialAccelEvent* SerialAccelEvent::get_instance()
{
    static SerialAccelEvent accel_event;
    return &accel_event;
}