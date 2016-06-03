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
	    uint8_t d;
	    char c;
	    int16_t numc;
	    bool parsed = false;

	    numc = port->available();
	    hal.console->printf("Number of bytes available on serial: %u\n", numc);

	    for (int16_t i = 0; i < numc; i++)
	    {        // Process bytes received

	        // read the next byte
	        c = (char)port->read();
	        hal.console->printf("%c",c);

	    }
	    return parsed;
}



//	// hal.console->printf("TEST Message AP_VisionPose_Jetson\n");
//
//	char buf[250];
//	unsigned int i = 0;
//
//    bool parsed = false;
//
//    char c = '0';
//
//    // port->available() > 0 &&
//	while(c != '#' && i < sizeof(buf)-1)
//	{
//		  hal.console->printf("%c",c);
//	      c = port->read();
//	      buf[i++] = c;
//	}
//
//	// buf[sizeof(buf)-1] = '\0';
//	buf[249] = '\0';
//
//	//hal.console->printf("Received Message: %s\n",buf);
//
//    return parsed;

// }
