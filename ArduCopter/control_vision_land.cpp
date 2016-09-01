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

	AP_VisionPose::VisionPose_State* current_state;

	current_state = vision_pose.getState();

	// hal.console->printf("%d %f %f %f\n",current_state->frame_number ,current_state->position.x, current_state->position.y, current_state->position.z);

    float target_roll, target_pitch;
    // float target_yaw_rate = 0.0f;

    float pilot_throttle_scaled = 0.0f;

	float posX_cm = 0.0f, posY_cm = 0.0f;
	float posZ_cm = 0.0f;

	float posX_cm_NED, posY_cm_NED, posZ_cm_NED;
	float posX_cm_NEU, posY_cm_NEU, posZ_cm_NEU;

	uint8_t marker_detected = 0;
	int frame_number = 0;

	float roll_angle, pitch_angle;

	// float yaw_rad = 0.0f;

	// yaw_rad = ((float)ahrs.yaw_sensor/100.0)*3.1415/180.0f;

    // initialize vertical speeds and acceleration
//    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
//    pos_control.set_accel_z(g.pilot_accel_z);

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

    if(marker_detected == 1)
    {
        posX_cm = current_state->position.x;
        posY_cm = current_state->position.y;
        posZ_cm = current_state->position.z;

    }
    else
    {
        posX_cm = 0.0f;
        posY_cm = 0.0f;
        posZ_cm = 0.0f;
    }

    posX_cm_NED = posX_cm*ahrs.cos_yaw()-posY_cm*ahrs.sin_yaw();
    posY_cm_NED = posX_cm*ahrs.sin_yaw()+posY_cm*ahrs.cos_yaw();
    float yaw = ahrs.get_yaw();
    posZ_cm_NED = posZ_cm;

    posX_cm_NED = -100.0f;
    posY_cm_NED = 0.0f;

    // get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
    // target_yaw_rate = 0.0f; // get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    float dt = pos_control.time_since_last_xy_update();

//    if (dt >= pos_control.get_dt_xy()) {
//        // sanity check dt
//        if (dt >= 0.2f) {
//            dt = 0.0f;
//        }
////        pos_control.set_alt_target(150.0);
////        pos_control.set_xy_target(0.0f,0.0f);
//    	pos_control._update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0, true, marker_detected, posX_cm_NED, posY_cm_NED, posZ_cm_NED);
//    }


    float pos_error = 0.0f;
    float current_position_x = 0.0f;
    float desired_position_x = 0.0f;
    float yaw = 0.0f;

    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

//    if(marker_detected == 1)
//    {
    	roll_angle = pos_control.get_roll();
        pitch_angle = pos_control.get_pitch();
//    }
//    else
//    {
//    	roll_angle = 0.0f;
//    	pitch_angle = 0.0f;
//    }

    // hal.console->printf("Euler Angles: %d %f %f %f %f\n", marker_detected, posX_cm_NED, posY_cm_NED, roll_angle, pitch_angle);

//    float max_angle = 200.0;
//
//    if(roll_angle > max_angle)
//    	roll_angle = max_angle;
//    if(roll_angle < -max_angle)
//    	roll_angle = -max_angle;
//
//    if(pitch_angle > max_angle)
//    	pitch_angle = max_angle;
//    if(pitch_angle < -max_angle)
//    	pitch_angle  = -max_angle;

//    if(marker_detected == 1)
//    	pos_control._update_z_controller(marker_detected, -posZ_cm_NED);
//    else
//    	pos_control.update_z_controller();

    // hal.console->printf("RP: %f %f\n", roll_angle, pitch_angle);

    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(roll_angle, pitch_angle, 0.0, get_smoothing_gain());
    // attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(roll_angle, pitch_angle, target_yaw_rate); // , get_smoothing_gain());
    // attitude_control.input_euler_angle_roll_pitch_yaw(roll_angle,pitch_angle,0.0,true);

    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    // CHECK THE SIGN OF X AND Y AS INPUT OF THE XY POSITION CONTROLLER
    // CHECK IF SET_SPEED_Z AND SET_ACCEL_Z ARE STILL NECESSARY

	if(cnt%2==0)
	{
		Log_Write_VisionPose_XY(marker_detected, frame_number, 0.0, 0.0, posX_cm_NED, posY_cm_NED, roll_angle, pitch_angle, yaw);
		// Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm);
	}
	cnt++;
}
