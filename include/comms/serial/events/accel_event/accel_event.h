//
// Created by mcleans on 9/30/23.
//

#ifndef ROBOT_CONTROLLER_ACCEL_EVENT_H
#define ROBOT_CONTROLLER_ACCEL_EVENT_H

#include "serial_events.h"

/**
 * Accelerometer event data
 */
typedef struct s_accel_event {
    int x_cws; /* X axis clockwise angle */
    int x_acws; /* X axis anticlockwise angle */
    int y_cws; /* Y axis clockwise angle */
    int y_acws; /* Y axis anticlockwise angle */
}s_accel_ev_t;

/**
 * Accelerometer Serial Event Class
 */
class SerialAccelEvent : public SerialCommsEvents {
private:
    /**
     * Event data
     */
    s_accel_ev_t *_ev = nullptr;

public:

    int emit(void* ev) override;

    void post() override;

    /**
     * @brief Get the instance object
     * 
     * @return SerialAccelEvent* 
     */
    static SerialAccelEvent* get_instance();
};

#endif //ROBOT_CONTROLLER_ACCEL_EVENT_H
