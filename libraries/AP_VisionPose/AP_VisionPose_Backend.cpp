/*
 * VisionPose_Backend.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Alessandro Benini
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_VisionPose.h"
#include "AP_VisionPose_Backend.h"

extern const AP_HAL::HAL& hal;

AP_VisionPose_Backend::AP_VisionPose_Backend(VisionPose &vision_pose) :
    _vision_pose(vision_pose)
{}

void AP_VisionPose_Backend::set_last_update_usec(uint32_t last_update, uint8_t instance)
{
	VisionPose::vision_pose_state &state = _vision_pose._state[instance];
    state.last_update_usec = last_update;
}

/*
  register a new backend with frontend, returning instance which
  should be used in publish_field()
 */
uint8_t AP_VisionPose_Backend::register_vision_pose(void) const
{
    return _vision_pose.register_vision_pose();
}
