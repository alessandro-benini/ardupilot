/*
 * VisionPose.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once

#include <inttypes.h>

#include <AP_HAL/AP_HAL.h>
#include "AP_VisionPose_Backend.h"

// For the moment the maximum number of vision pose sensors is limited to 1.
#define VISION_POSE_MAX_INSTANCES 1
#define VISION_POSE_MAX_BACKEND   1

class VisionPose
{
friend class AP_VisionPose_Backend;
public:

	VisionPose();

	bool init();

	bool read();

    // backend objects
	AP_VisionPose_Backend *_backends[VISION_POSE_MAX_BACKEND];
    uint8_t     _backend_count;

    // number of registered vision_pose sensors.
    uint8_t     _vision_pose_count;

    // Returns true if the vision pose sensor has been configured
    bool configured(void);

    // Functions to access the data members
    const Vector3f &get_attitude_f(uint8_t i) const { return _state[i].attitude_f; }
    const Vector3f &get_attitude_r(uint8_t i) const { return _state[i].attitude_r; }
    const Vector3f &get_position_f(uint8_t i) const { return _state[i].position_f; }
    const Vector3f &get_position_r(uint8_t i) const { return _state[i].position_r; }

private:

    /// Register a new vision_pose driver, allocating an instance number
    ///
    /// @return number of vision_pose instances
    uint8_t register_vision_pose(void);

    // load backend drivers
    bool _add_backend(AP_VisionPose_Backend *backend, const char *name, bool external);
    void _detect_backends(void);

	struct vision_pose_state {

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

		// Check if the measure is healty (for example after CRC checck)
		bool healthy;

        // When we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

	} _state[VISION_POSE_MAX_INSTANCES];


};
