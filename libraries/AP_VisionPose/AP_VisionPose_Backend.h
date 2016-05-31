/*
 * AP_VisionPose_Backend.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once

#include "AP_VisionPose.h"

class AP_VisionPose_Backend
{
public:
	AP_VisionPose_Backend(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port);

    // we declare a virtual destructor so that GPS drivers can
    // override with a custom destructor if need be.
    virtual ~AP_VisionPose_Backend(void) {}

    // The read() method is the only one needed in each driver. It
    // should return true when the backend has successfully received a
    // valid packet from the GPS.
    virtual bool read() = 0;

protected:
    AP_HAL::UARTDriver *port;              		///< UART we are attached to
    AP_VisionPose &vision_pose;                 ///< access to frontend (for parameters)
    AP_VisionPose::VisionPose_State &state;     ///< public state for this instance

};

