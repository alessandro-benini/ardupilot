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

bool first_time = false;

bool first_movement = true;
uint32_t start_time = 0;

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
    pos_control.set_alt_target(150.0);
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

	float Kp = 0.1;

	AP_VisionPose::VisionPose_State* current_state;
	current_state = vision_pose.getState();

    //float pos_error_x = 0.0f;
    //float desired_position_x = 0.0f;

    //float pos_error_y = 0.0f;
    //float desired_position_y = 0.0f;

    //float pos_error_z = 0.0f;
    //float desired_position_z = 100.0f;

    float yaw = 0.0f;

    float target_roll = 0.0f, target_pitch = 0.0f;

    float target_yaw_rate = 0.0f;

    float pilot_throttle_scaled = 0.0f;

	float posX_cm = 0.0f, posY_cm = 0.0f;
	float posX_cm_B = 0.0f, posY_cm_B = 0.0f;

	float posZ_cm = 0.0f;

	float posX_cm_NED, posY_cm_NED, posZ_cm_NED;
	float pos_error_x_BODY, pos_error_y_BODY;

	uint8_t marker_detected = 0;
	int frame_number = 0;

	float yaw_vision = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

	set_auto_yaw_mode(AUTO_YAW_HOLD);

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    // update_simple_mode();

    frame_number = current_state->frame_number;
    marker_detected = current_state->marker_detected;

    // posX_cm, posY_cm, posZ_cm are expressed in marker reference frame
    if(marker_detected == 1)
    {
        posX_cm = current_state->position.x;
        posY_cm = current_state->position.y;
        posZ_cm = current_state->position.z;
        yaw_vision = (current_state->attitude.z)*(3.1415/180.0f);

    }
    else
    {
        posX_cm = 0.0f;
        posY_cm = 0.0f;
        // posZ_cm = 0.0f;
        // yaw_vision = 0.0f;
    }

    // Calculate error on marker reference frame
    //pos_error_x = desired_position_x - posX_cm;
    //pos_error_y = desired_position_y - posY_cm;

    // Convert error to Body frame
    posX_cm_B = posX_cm*cos(yaw_vision)+posY_cm*sin(yaw_vision);
    posY_cm_B = -posX_cm*sin(yaw_vision)+posY_cm*cos(yaw_vision);

    pos_error_x_BODY = 0.0 - posX_cm_B;
    pos_error_y_BODY = 0.0 - posY_cm_B;

    target_roll  = pos_error_y_BODY*Kp;
    target_pitch = -pos_error_x_BODY*Kp;

    // pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    float max_angle = 300.0;

    if(target_roll > max_angle)
    	target_roll = max_angle;
    if(target_roll < -max_angle)
    	target_roll = -max_angle;

    if(target_pitch > max_angle)
    	target_pitch = max_angle;
    if(target_pitch < -max_angle)
    	target_pitch  = -max_angle;

    if(marker_detected == 1)
    	pos_control._update_z_controller(marker_detected, -posZ_cm);
    else
    	pos_control.update_z_controller();

    // hal.console->printf("RP: %f %f %f %f %f %f %f\n", posX_cm, posY_cm, posX_cm_B,posY_cm_B,target_roll,target_pitch,yaw_vision); // ,posZ_cm); // , pos_error_x_BODY, pos_error_y_BODY, target_roll, target_pitch);
    // hal.console->printf("RP: %f %f %f %f %f %f %f\n", pos_error_x, pos_error_y,pos_error_x_BODY,pos_error_y_BODY,target_roll,target_pitch,posZ_cm); // , pos_error_x_BODY, pos_error_y_BODY, target_roll, target_pitch);

    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, 0.0, get_smoothing_gain());
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0.0); // , get_smoothing_gain());
    // attitude_control.input_euler_angle_roll_pitch_yaw(target_roll,target_pitch,-900.0f,true);

    // attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    // CHECK THE SIGN OF X AND Y AS INPUT OF THE XY POSITION CONTROLLER
    // CHECK IF SET_SPEED_Z AND SET_ACCEL_Z ARE STILL NECESSARY

	if(cnt%2==0)
	{
		Log_Write_VisionPose_XY(marker_detected, frame_number, posX_cm, posY_cm, posX_cm_B, posY_cm_B, target_roll, target_pitch, yaw);
		// Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm);
	}
	cnt++;
}
