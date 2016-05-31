/*
 * AP_VisionPose_Jetson.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once

#include "AP_VisionPose.h"
#include "AP_VisionPose_Backend.h"

class AP_VisionPose_Jetson : public AP_VisionPose_Backend {
public:
	AP_VisionPose_Jetson(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port);
	bool read(void);
};
