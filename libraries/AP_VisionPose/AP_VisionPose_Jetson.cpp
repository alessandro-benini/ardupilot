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
	    char c;
	    int16_t numc;
	    bool parsed = false;

	    numc = port->available();
	    hal.console->printf("Number of bytes available on serial: %u", numc);

	    char msg[200];

	    bool found_header = false;
	    uint8_t j = 0;

	    // Need to check overflow
	    for (int16_t i = 0; i < numc; i++)
	    {
	        // read the next byte
	        c = (char)port->read();

	        // Start looking for the JSON string
	        if(c == '{')
	        {
	        	// I've found the header
	        	msg[j] = c;
	        	found_header = true;
	        }
	        else
	        {
		        if(found_header)
		        {
		        	j += 1;
		        	if(c != '}')
		        		msg[j] = c;
		        	else
		        	{
		        		msg[j] = '}';
		        		msg[j+1] = '\0';
		        		hal.console->printf("%s",msg);
		        		parsed = true;
		        	}
		        }
	        }

	    }

	    return parsed;
}
