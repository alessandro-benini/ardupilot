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

// float target_roll = 0.0f, target_pitch = 0.0f;
// float target_yaw_rate = 0.0f;
// float pilot_throttle_scaled = 0.0f;

float altitude_error_cm = 0.0f;
float posZ_cm = 0.0f;
float target_climb_rate_cm_s = 0.0f;
uint8_t marker_detected = 0;
int frame_number = 0;

// Counter to log the data
int cnt = 0;

Vector3f posvel_pos_target_cm(0.0f,0.0f,-100.0f);

int16_t base_throttle = 0;

float total_throttle = 0.0f;

bool Copter::vision_land_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    // pos_control.set_alt_target(-100.0);

    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(-100.0);
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    base_throttle = channel_throttle->get_control_in();

    // stop takeoff if running
    takeoff_stop();

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

	float posX_cm = 0.0f, posY_cm = 0.0f;
	float x_error_cm = 0.0f, y_error_cm = 0.0f;
	float x_rate = 0.0f, y_rate = 0.0f;

    float pilot_throttle_scaled;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    frame_number = vision_pose.get_frame_number();
    marker_detected = vision_pose.is_marker_detected();

	if(marker_detected)
	{
		posZ_cm = vision_pose.get_z_position();
		// get pilot desired climb rate
		altitude_error_cm = (-100.0 - posZ_cm);
		target_climb_rate_cm_s = 0.75*altitude_error_cm;
		// Make sure that the climb rate is bounded
		target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	}
	else
		// If I didn't detect the marker, I simply keep the current altitude.
		target_climb_rate_cm_s = 0.0;

	total_throttle = (float)base_throttle + target_climb_rate_cm_s;

//    if(marker_detected && vision_pose.is_healty())
//    {
//    	posX_cm = vision_pose.get_x_position();
//    	posY_cm = vision_pose.get_y_position();
//    	x_error_cm = 0.0 - posX_cm;
//    	y_error_cm = 0.0 - posY_cm;
//
//    	x_rate = 0.3*x_error_cm;
//    	y_rate = 0.3*y_error_cm;
//
//    	if(x_rate>20.0)
//    		x_rate = 20.0;
//    	if(x_rate < -20.0)
//    		x_rate = -20.0;
//
//    	if(y_rate>20.0)
//    		y_rate = 20.0;
//    	if(y_rate < -20.0)
//    		y_rate = -20.0;
//
//        pos_control.set_desired_velocity_xy(x_rate,y_rate);
//    }
//    else
//    	pos_control.set_desired_velocity_xy(0.0f,0.0f);
//
//    pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, ekfNavVelGainScaler, true);
//    // pos_control.update_vel_controller_xy(ekfNavVelGainScaler);
//
//    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    //attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(pos_control.get_roll(), pos_control.get_pitch(), 0.0, get_smoothing_gain());
    attitude_control.set_throttle_out(total_throttle, true, g.throttle_filt);

	if(cnt%2==0)
		// Log_Write_VisionPose_XY(marker_detected, frame_number, posX_cm, posY_cm, x_error_cm, y_error_cm, x_rate, y_rate);
		Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm, altitude_error_cm, total_throttle);
	cnt++;

}
