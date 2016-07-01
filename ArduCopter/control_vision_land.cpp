/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#include <unistd.h>

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
extern const AP_HAL::HAL& hal;

static bool land_with_vision;

static uint32_t land_start_time;
static bool land_pause;

int cnt = 0;

float x = 0.0f;
float y = 0.0f;
float z = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

bool Copter::vision_land_init(bool ignore_checks)
{

	// set target to stopping point
	Vector3f stopping_point(0.0,0.0,0.0);
	wp_nav.get_loiter_stopping_point_xy(stopping_point);
	wp_nav.init_loiter_target(stopping_point);

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    land_start_time = millis();

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    return true;
}

/**
 * The landing phase will be made in two steps:
 * 1) alignment of the quadrotor with the platform (x,y,z,yaw);
 * 2) landing decreasing the altitude
 * Since the quadrotor can drift during the landing phase, the two steps must be executed consecutively at each step.
 */
void Copter::vision_land_run()
{
	++cnt;
	x = vision_pose.get_x_position();
	y = vision_pose.get_y_position();
	z = vision_pose.get_z_position();
	yaw = vision_pose.get_yaw();

	hal.console->printf("Yaw: %f\n",yaw);
}
