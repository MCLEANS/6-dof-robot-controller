#include "serial_events.h"
#include "assert.h"

#define  MAX_SERIAL_COMMS_EVENTS_NUMBER 256

QueueHandle_t* SerialCommsEventsQueue::get_default_instance()
{
    static QueueHandle_t serial_comms_events_queue = xQueueCreate( MAX_SERIAL_COMMS_EVENTS_NUMBER, sizeof( SerialCommsEvents) );
    assert(serial_comms_events_queue != NULL);
    return &serial_comms_events_queue;
}