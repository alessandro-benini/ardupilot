/*
 * VisionPose.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_VisionPose.h"

extern const AP_HAL::HAL &hal;

////////////////
////////////////
// Note: AP_GPS::update(void) is where it is selected the primary GPS (based on the estimation quality)

/// Startup initialisation.
void AP_VisionPose::init(DataFlash_Class *dataflash, const AP_SerialManager& serial_manager)
{
    _DataFlash = dataflash;
    primary_instance = 0;

    // search for serial ports with gps protocol
    _port[0] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Jetson_TK1, 0);
    _last_instance_swap_ms = 0;
}

