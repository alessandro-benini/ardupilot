/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
extern const AP_HAL::HAL& hal;

bool Copter::vision_land_init(bool ignore_checks)
{
    return true;
}

void Copter::vision_land_run()
{
	// Here it comes the control system for landing the Helicopter.
	hal.console->printf("Pos X: %f\n",vision_pose.get_x_position());

}
