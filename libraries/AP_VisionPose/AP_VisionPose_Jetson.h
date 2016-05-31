/*
 * AP_VisionPose_Jetson.h
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#pragma once


#include "AP_VisionPose.h"
#include "AP_VisionPose_Backend.h"

class AP_VisionPose_Jetson : public AP_VisionPose_Backend
{
public:

	static constexpr const char *name = "JETSON_POSE";

	virtual ~AP_VisionPose_Jetson();

	bool init() override;
	void read() override;

private:

	AP_VisionPose_Jetson(VisionPose &vision_pose, AP_AK8963_BusDriver *bus,
                      uint32_t dev_id);

	void _update();
};
