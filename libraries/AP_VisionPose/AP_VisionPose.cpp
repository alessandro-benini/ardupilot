/*
 * VisionPose.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_VisionPose.h"

extern AP_HAL::HAL& hal;

bool VisionPose::init()
{
    return true;
}

//  Register a new compass instance
//
uint8_t VisionPose::register_vision_pose(void)
{
    if (_vision_pose_count == VISION_POSE_MAX_INSTANCES) {
        AP_HAL::panic("Too many vision_pose instances");
    }
    return _vision_pose_count++;
}

bool VisionPose::_add_backend(AP_VisionPose_Backend *backend, const char *name, bool external)
{
    if (name != nullptr) {
        hal.console->printf("%s: %s Vision Pose %sdetected\n", name,
                            external ? "External" : "Onboard",
                            backend == nullptr ? "not " : "");
    }

    if (!backend) {
        return false;
    }

    if (_backend_count == VISION_POSE_MAX_BACKEND) {
        AP_HAL::panic("Too many vision_pose backends");
    }

    _backends[_backend_count++] = backend;

    return true;
}

/*
  detect available backends for this board
 */
void VisionPose::_detect_backends(void)
{

	// TODO: Check if _detect_backends is really necessary in my case.
}

bool VisionPose::read(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }
    for (uint8_t i=0; i < VISION_POSE_MAX_INSTANCES; i++) {
        _state[i].healthy = (AP_HAL::millis() - _state[i].last_update_ms < 500);
    }

    // TODO: Check the prototype of the function helaty() for VisionPose
    return true; // healthy();
}

