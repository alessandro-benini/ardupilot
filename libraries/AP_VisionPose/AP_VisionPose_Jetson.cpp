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
					// hal.console->printf("%s",msg);
					if(decode_JSON(msg))
						parsed = true;
				}
			}
		}
	}
	return parsed;
}

bool AP_VisionPose_Jetson::decode_JSON(char JSON_STRING[])
{

	int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[50];

	jsmn_init(&p);
	r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
	if (r < 0) {
		printf("***Failed to parse JSON: %d***\n", r);
		return false;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		printf("***Object expected***\n");
		return false;
	}

	// Loop over all keys of the root object
	// The structure of the JSON content is fixed.
	for (i = 1; i < r; i++) {
		if (jsoneq(JSON_STRING, &t[i], "POSE") == 0) {
			int j = 0;
			if (t[i+1].type != JSMN_ARRAY) {
				continue; /* We expect groups POSE to be an array of strings */
			}

            // This string will contain the string to be converted in float
            char requested_data[100];

			jsmntok_t *g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.x = atof(requested_data);

            j = 1;
			g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.y = atof(requested_data);

            j = 2;
			g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.z = atof(requested_data);

            j = 3;
			g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.roll = atof(requested_data);

            j = 4;
			g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.pitch = atof(requested_data);

            j = 5;
			g = &t[i+j+2];
            sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
            state.yaw = atof(requested_data);

		}
		else {
			printf("***Unexpected key: %.*s***\n", t[i].end-t[i].start,
					JSON_STRING + t[i].start);
		}
	}

	return true;
}
