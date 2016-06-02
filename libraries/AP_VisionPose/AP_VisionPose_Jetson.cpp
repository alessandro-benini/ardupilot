/*
 * AP_VisionPose_Jetson.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include "AP_VisionPose.h"
#include "AP_VisionPose_Jetson.h"

extern const AP_HAL::HAL& hal;

AP_VisionPose_Jetson::AP_VisionPose_Jetson(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port) :
		AP_VisionPose_Backend(_vision_pose, _state, _port) {}

bool AP_VisionPose_Jetson::read(void)
{

	char buf[250];
	unsigned int i = 0;

    bool parsed = false;

    char c = '#';

	while(port->available() > 0 && c != '\n' && i < sizeof(buf)-1)
	{
	      c = port->read();
	      buf[i++] = c;
	}

	buf[sizeof(buf)-1] = '\0';

	hal.console->printf("Received Message: %s\n",buf);

    return parsed;
}
