/*
 * VisionPose_Backend.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_VisionPose.h"
#include "AP_VisionPose_Backend.h"

extern const AP_HAL::HAL& hal;

AP_VisionPose_Backend::AP_VisionPose_Backend(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port) :
	VisionPose(_vision_pose),
	port(_port),
    state(_state) {}
