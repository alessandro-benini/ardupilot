/*
 * VisionPose.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_SerialManager/AP_SerialManager.h>

// For the moment the maximum number of vision pose sensors is limited to 1.
#define VISION_POSE_MAX_INSTANCES 1
#define VISION_POSE_MAX_BACKEND   1

class DataFlash_Class;
class AP_VisionPose_Backend;

class AP_VisionPose
{
public:

    // constructor
	AP_VisionPose();

    /// Startup initialisation.
	void init(AP_HAL::UARTDriver *port, const char *name);

    /// Update VisionPose state based on possible bytes received from the module.
    /// This routine must be called periodically (typically at 10Hz or
    /// more) to process incoming data.
    void update(void);

	struct VisionPose_State {

		// Filtered attitude vector (Roll, Pitch, Yaw) - NED frame
		AP_Vector3f attitude_f;

		// Filtered position vector - NED frame
		AP_Vector3f position_f;

		// Check if the current pose estimation is based on the actual detection
		// of the marker or is based on the estimation using the Kalman Filter prediction.
		bool marker_detected;

		// Raw attitude vector (Roll, Pitch, Yaw) - NED frame
		AP_Vector3f attitude_r;

		// Raw position vector - NED frame
		AP_Vector3f position_r;

		// Check if the measure is healthy (for example after CRC check)
		bool healthy;

        // When we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

	};

	static const struct AP_Param::GroupInfo var_info[];

    // configuration parameters
    uint32_t _last_instance_swap_ms;

private:

    VisionPose_State state;
    AP_VisionPose_Backend *driver;
    AP_HAL::UARTDriver *_port;

};

