#ifndef SERIAL_COMMS_EVENTS_H
#define SERIAL_COMMS_EVENTS_H

#include <FreeRTOS.h>
#include <queue.h>

/**
 * Queue to handle data to be sent over the serial interface
 */
class SerialCommsEventsQueue {
    
public :
    /**
     *
     * @return default instance of serial comms events queue
     */
    static QueueHandle_t* get_default_instance();
};

/**
 * Events to be sent over the serial port
 */
class SerialCommsEvents {

public:
    /**
     * Emit the event to the SerialCommsEventsQueue
     * @param[in] ev pointer to the event data to emit
     * @return 0 on success and -1 on failure
     */
    virtual int emit(void* ev) = 0;

    /**
     * Post the event over the Serial Interface
     */
    virtual void post() = 0;

};

#endif