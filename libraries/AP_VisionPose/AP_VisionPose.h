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
	AP_VisionPose() {
		AP_Param::setup_object_defaults(this, var_info);
    }

    /// Startup initialisation.
    void init(DataFlash_Class *dataflash, const AP_SerialManager& serial_manager);

    /// Update GPS state based on possible bytes received from the module.
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

		// Check if the measure is healty (for example after CRC checck)
		bool healthy;

        // When we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

	};

	static const struct AP_Param::GroupInfo var_info[];

    // dataflash for logging, if available
    DataFlash_Class *_DataFlash;

    // configuration parameters
    uint32_t _last_instance_swap_ms;

private:

    VisionPose_State state[VISION_POSE_MAX_INSTANCES];
    AP_VisionPose_Backend *drivers[VISION_POSE_MAX_INSTANCES];
    AP_HAL::UARTDriver *_port[VISION_POSE_MAX_INSTANCES];

    /// primary GPS instance
    uint8_t primary_instance:1;

    /// number of GPS instances present
    uint8_t num_instances:1;

    // which ports are locked
    uint8_t locked_ports:1;


};

