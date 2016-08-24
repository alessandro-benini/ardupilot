/*
 * AP_VisionPose_Jetson.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include "AP_VisionPose.h"
#include "AP_VisionPose_Jetson.h"
#include <string>

extern const AP_HAL::HAL& hal;

AP_Relay relay;

float x_ned = 0.0f, y_ned = 0.0f, z_ned = 0.0f;
uint8_t prev_frame_number = 0;

AP_VisionPose_Jetson::AP_VisionPose_Jetson(AP_VisionPose &_vision_pose, AP_VisionPose::VisionPose_State &_state, AP_HAL::UARTDriver *_port) :
		AP_VisionPose_Backend(_vision_pose, _state, _port)
{
	relay.init();
}

bool AP_VisionPose_Jetson::read(void)
{
	// Toggle relay pin 0 high for starting measuring
	// https://github.com/ArduPilot/ardupilot/issues/785
	// http://ardupilot.org/copter/docs/common-pixhawk-overview.html

	// relay.on(0);
	char c;
	int16_t numc;
	bool parsed = false;

	bool first_char = false;

	numc = port->available();

	char msg[90];

	char check = '\0';
	int head = 0;
	// hal.console->print("$");

	int idx = 0;

	double start = 0.0, end = 0.0;

	for (int16_t i = 0; i < numc; i++)
	{

		c = (char)port->read();

		// hal.console->print(c);

		serial_buffer.push_back(c);
		if (c == '{')
		{
			head = serial_buffer.get_tail_position();
		}
		if (c == '}')
		{
			// hal.console->print("\r\n");
			int index = head - serial_buffer.get_head_position();
			check = serial_buffer.peek(index);
			if (check == '{')
			{
				do {
					check = serial_buffer.peek(index);
					msg[idx] = check;
					++idx;
					++index;
				} while (check != '}');

				msg[idx] = '\0';

				if(decode_JSON(msg))
					parsed = true;

				// Reinitialize char array
				msg[0] = '\0';
				idx = 0;

			}
		}
	}
	// Toggle relay pin 0 low for stopping measuring
	// relay.off(0);

	return parsed;
}

bool AP_VisionPose_Jetson::decode_JSON(char JSON_STRING[])
{
	// hal.console->printf("Parsing %s\n",JSON_STRING); // : %s\n",JSON_STRING);
	int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[50];

	jsmn_init(&p);
	r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
	if (r < 0) {
		hal.console->printf("***Failed to parse JSON: %d***\n", r);
		return false;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		hal.console->printf("***Object expected***\n");
		return false;
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		if (jsoneq(JSON_STRING, &t[i], "POSE") == 0)
		{
			int j = 0;
			if (t[i+1].type != JSMN_ARRAY) {
				hal.console->printf("***POSE has to be an array of strings***\n");
				continue; /*  */
			}

			char requested_data[30];

			jsmntok_t *g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			state.marker_detected = atoi(requested_data);

			j = 1;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			state.frame_number = atoi(requested_data);

			if(state.frame_number != prev_frame_number)
				state.healthy = true;
			else
				state.healthy = false;

			prev_frame_number = state.frame_number;

			j = 2;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			x_ned = atof(requested_data);

			j = 3;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			y_ned = atof(requested_data);

			j = 4;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			z_ned = atof(requested_data);

			// Conversion from mm to cm.
			state.x = x_ned / 10.0f;
			state.y = y_ned / 10.0f;
			state.z = z_ned / 10.0f;

			j = 5;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			state.roll = atof(requested_data);

			j = 6;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			state.pitch = atof(requested_data);

			j = 7;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			state.yaw = atof(requested_data);

			state.last_update_msec =  AP_HAL::millis();
			state.last_update_usec =  state.last_update_msec / 1000.0f;

			i += t[i+1].size + 1;
		}
		else
			hal.console->printf("Unexpected key: %.*s\n", t[i].end-t[i].start,JSON_STRING + t[i].start);
	}

	return true;
}
