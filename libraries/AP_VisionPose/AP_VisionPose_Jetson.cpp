/*
 * AP_VisionPose_Jetson.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include "AP_VisionPose.h"
#include "AP_VisionPose_Jetson.h"

// extern const AP_HAL::HAL& hal;

AP_VisionPose_Jetson::AP_VisionPose_Jetson(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port) :
		AP_VisionPose_Backend(_vision_pose, _state, _port) {}

bool AP_VisionPose_Jetson::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    numc = port->available();

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();

    return parsed;
}
