/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// This is the first version of the vision_land based controller.
// The idea is to align the quadrotor with the marker using the vision based yaw estimation
// while keeping the current altitude

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
extern const AP_HAL::HAL& hal;

// Counter to log the data
int cnt = 0;

bool Copter::vision_land_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }

    pos_control.init_xy_controller(true);

    pos_control.set_speed_xy(20);
    pos_control.set_accel_xy(10);
    pos_control.set_jerk_xy_to_default();
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);
    pos_control.calc_leash_length_xy();
    pos_control.calc_leash_length_z();

    // initialise position and desired velocity
    pos_control.set_alt_target(130.0);
    pos_control.set_xy_target(0.0f,0.0f);

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
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;

	float posX_cm = 0.0f, posY_cm = 0.0f;
	float posZ_cm = 0.0f;

	float posX_cm_NED, posY_cm_NED;

	uint8_t marker_detected = 0;
	int frame_number = 0;

	float roll_angle, pitch_angle;

	float yaw_rad = 0.0f;

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

    posX_cm = vision_pose.get_x_position();
    posY_cm = vision_pose.get_y_position();
    posZ_cm = vision_pose.get_z_position();

    // get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    yaw_rad = ((float)ahrs.yaw_sensor/100.0)*3.1415/180.0f;

    posX_cm_NED = posX_cm*cos(yaw_rad)-posY_cm*sin(yaw_rad);
    posY_cm_NED = posY_cm*sin(yaw_rad)+posY_cm*cos(yaw_rad);

    float dt = pos_control.time_since_last_xy_update();

    //if (dt >= pos_control.get_dt_xy()) {
    pos_control._update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, 1.0, false, marker_detected, posX_cm_NED, posY_cm_NED);
    //}

    // hal.console->printf("Euler Angles: %f %f %f %f %f\n", posX_cm, posY_cm, posX_cm_NED, posY_cm_NED, yaw_rad);

    roll_angle = pos_control.get_roll();
    pitch_angle = pos_control.get_pitch();

    float max_angle = 150.0;

    if(roll_angle > max_angle)
    	roll_angle = max_angle;
    if(roll_angle < -max_angle)
    	roll_angle = -max_angle;

    if(pitch_angle > max_angle)
    	pitch_angle = max_angle;
    if(pitch_angle < -max_angle)
    	pitch_angle  = -max_angle;

    // hal.console->printf("RPY: %f %f %f\n", pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate, get_smoothing_gain());
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(roll_angle, pitch_angle, target_yaw_rate, get_smoothing_gain());

    pos_control._update_z_controller(marker_detected, -posZ_cm);

    // CHECK THE SIGN OF X AND Y AS INPUT OF THE XY POSITION CONTROLLER
    // CHECK IF SET_SPEED_Z AND SET_ACCEL_Z ARE STILL NECESSARY

	if(cnt%2==0)
	{
		Log_Write_VisionPose_XY(marker_detected, frame_number, posX_cm, posY_cm, posX_cm_NED, posY_cm_NED, roll_angle, pitch_angle, yaw_rad);
		Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm);
	}
	cnt++;
}
