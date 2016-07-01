/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// This is the first version of the vision_land based controller.
// The idea is to align the quadrotor with the marker using the vision based yaw estimation
// while keeping the current altitude

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
// Desired yaw will be probably always zero.
float desired_yaw = 0.0f;
float current_yaw = 0.0f;
float yaw_error = 0.0f;
float target_yaw_rate = 0.0f;
float Kp_yaw = 0.2;

uint8_t marker_detected = 0;

bool Copter::vision_land_init(bool ignore_checks)
{
	// TODO: Double check all the initializations in this function

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

	// This is the current yaw estimation (if the marker is detected)
	if(vision_pose.is_marker_detected())
	{
		marker_detected = 1;
		current_yaw = vision_pose.get_yaw();
	}
	else
	{
		current_yaw = ahrs.yaw_sensor;
		marker_detected = 0;
	}

	// I calculate the yaw current yaw error
	yaw_error = desired_yaw - current_yaw;

	// The yaw_error is basically a rate (finite differences) v_yaw = (d_yaw - c_yaw)/dT
	// Therefore I have a reference for the yaw rate controller.
	target_yaw_rate = Kp_yaw*yaw_error;

	// Now I can apply the yaw rate reference to the yaw rate controller
    // Call the attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, target_yaw_rate);

    // get pilot desired climb rate
    float target_climb_rate = 0.0f;
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // call position controller
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();

    Log_Write_VisionPose(marker_detected,x,y,z,current_yaw,yaw_error,target_yaw_rate);

}
