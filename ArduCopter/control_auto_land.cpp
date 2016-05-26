/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::auto_land_init(bool ignore_checks)
{
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::auto_land_run()
{
}
