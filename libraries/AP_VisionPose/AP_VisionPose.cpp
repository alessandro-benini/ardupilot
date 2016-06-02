/*
 * VisionPose.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_VisionPose.h"
#include "AP_VisionPose_Backend.h"
#include "AP_VisionPose_Jetson.h"

extern const AP_HAL::HAL &hal;

////////////////
////////////////
// Note: AP_GPS::update(void) is where it is selected the primary GPS (based on the estimation quality)

AP_VisionPose::AP_VisionPose() : _last_instance_swap_ms(0){}

/// Startup initialization.
void AP_VisionPose::init(AP_HAL::UARTDriver *port)
{
    _port = port;
    _port->begin(115200,256,256);
    _last_instance_swap_ms = 0;
    // _DataFlash = dataflash;

    driver = new AP_VisionPose_Jetson(*this, state, _port);

}

void AP_VisionPose::update(void)
{
	_last_instance_swap_ms = 0;
}

