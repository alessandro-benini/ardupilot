/*
 * VisionPose_Backend.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once

#include "AP_VisionPose.h"

class VisionPose;
class AP_VisionPose_Backend
{
public:
	AP_VisionPose_Backend(VisionPose &vision_pose);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_VisionPose_Backend(void) {}

    // initialize the vision pose sensor
    virtual bool init(void) = 0;

    // read sensor data
    virtual void read(void) = 0;

    // accumulate a reading from the sensor. Optional in backends
    virtual void accumulate(void) {};

protected:

    void set_last_update_usec(uint32_t last_update, uint8_t instance);

    // register a new vision_pose instance with the frontend
    uint8_t register_vision_pose(void) const;

    // access to frontend
    VisionPose &_vision_pose;
};
