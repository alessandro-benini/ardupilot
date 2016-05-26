/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::vision_land_init(bool ignore_checks)
{
	hal.uartE->begin(115200);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::vision_land_run()
{
	hal.uartE->printf("Hello on UART E - at %.3f seconds\n",AP_HAL::millis()*0.001f);
}
