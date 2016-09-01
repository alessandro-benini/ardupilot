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
    	    port(_port),
    	    vision_pose(_vision_pose),
    	    state(_state)
{
	pthread_mutex_init(&visionpose_mutex, NULL);
}

void AP_VisionPose_Backend::setState(AP_VisionPose::VisionPose_State* _state)
{
	// I don't have any pointer inside the structure, so I can do bit-wise copy
	pthread_mutex_lock(&visionpose_mutex);
	state.frame_number = _state->frame_number;
	state.healthy = _state->healthy;
	state.last_update_msec = _state->last_update_msec;
	state.last_update_usec = _state->last_update_usec;
	state.marker_detected = _state->marker_detected;
	state.position = _state->position;
	state.attitude = _state->attitude;
	pthread_mutex_unlock(&visionpose_mutex);
}

AP_VisionPose::VisionPose_State* AP_VisionPose_Backend::getState()
{
	pthread_mutex_lock(&visionpose_mutex);
	return &state;
	pthread_mutex_unlock(&visionpose_mutex);
}
