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
float vx_ned = 0.0f, vy_ned = 0.0f, vz_ned = 0.0f;

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

	char msg[150];

	char check = '\0';
	int head = 0;
	// hal.console->print("$");

	int idx = 0;

	double start = 0.0, end = 0.0;

//	for (int16_t i = 0; i < numc; i++)
//	{
//		c = (char)port->read();
//		if(c =='{')
//		{
//			idx = 0;
//			msg[idx++] = '{';
//		}
//		else if(c=='}')
//		{
//			msg[idx] = '}';
//			msg[++idx] = '\0';
//
//			hal.console->printf("String: %s\n",msg);
//			if(decode_JSON(msg))
//				parsed = true;
//			msg[0] = '\0';
//		}
//		else
//			msg[idx++] = c;
//	}

	for (int16_t i = 0; i < numc; i++)
	{

		c = (char)port->read();

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

			char requested_data[50];

			jsmntok_t *g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			current_state.marker_detected = atoi(requested_data);

			j = 1;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			current_state.frame_number = atoi(requested_data);

			if(current_state.frame_number != prev_frame_number)
				current_state.healthy = true;
			else
				current_state.healthy = false;

			prev_frame_number = current_state.frame_number;

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
			current_state.position.x = x_ned / 10.0f;
			current_state.position.y = y_ned / 10.0f;
			current_state.position.z = z_ned / 10.0f;

			j = 5;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			vx_ned = atof(requested_data);

			j = 6;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			vy_ned = atof(requested_data);

			j = 7;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			vz_ned = atof(requested_data);

			// Conversion from mm to cm.
			current_state.velocity.x = vx_ned / 10.0f;
			current_state.velocity.y = vy_ned / 10.0f;
			current_state.velocity.z = vz_ned / 10.0f;

			j = 8;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			current_state.attitude.x = atof(requested_data);

			j = 9;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			current_state.attitude.y = atof(requested_data);

			j = 10;
			g = &t[i+j+2];
			sprintf(requested_data, "%.*s", g->end - g->start, JSON_STRING + g->start);
			current_state.attitude.z = atof(requested_data);

			current_state.last_update_msec = AP_HAL::millis();
			current_state.last_update_usec = current_state.last_update_msec / 1000.0f;

			i += t[i+1].size + 1;

			// hal.console->printf("Parsing %f %f\n",current_state.position.x, current_state.position.y);

			setState(&current_state);
		}
		else
			hal.console->printf("Unexpected key: %.*s\n", t[i].end-t[i].start,JSON_STRING + t[i].start);
	}

	return true;
}
