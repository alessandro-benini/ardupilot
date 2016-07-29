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

//float x = 0.0f;
//float y = 0.0f;
//float z = 0.0f;
//float roll = 0.0f;
//float pitch = 0.0f;
//// Desired yaw will be probably always zero.
//float desired_yaw = 0.0f;
//float current_yaw = 0.0f;
//float yaw_error = 0.0f;
//float target_yaw_rate = 0.0f;
//float Kp_yaw = 0.2;
//
//int frame_number = 0;
//uint8_t marker_detected = 0;
//
//float target_climb_rate = 0.0f;
//float altitude_error = 0.0f;
//
int cnt = 0;

bool Copter::vision_land_init(bool ignore_checks)
{

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    // pos_control.set_alt_target(100.0);

    hal.console->printf("***Init vision landing mode***");
//
//    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // Initialize the desired position for hovering (100 cm above the marker)
    pos_control.set_alt_target(100.0);
    pos_control.set_desired_velocity_z(0.0);
//
//    // stop takeoff if running
//    takeoff_stop();

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
    float target_yaw_rate;
    float pilot_throttle_scaled;

    float altitude_error_cm;
    float posZ_cm;
    float target_climb_rate_cm_s = 0.0f;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speeds and acceleration // Limits to the max velocity and acceleration along z axis
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    uint8_t marker_detected = vision_pose.is_marker_detected();
    int frame_number = vision_pose.get_frame_number();

    posZ_cm = vision_pose.get_z_position();

	// get pilot desired climb rate
    altitude_error_cm = (pos_control.get_alt_target() - posZ_cm);
    target_climb_rate_cm_s = 0.50*altitude_error_cm;

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

	// call position controller
	pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate_cm_s, G_Dt, false);
	pos_control.update_z_vel_controller();

	if(cnt%4==0)
		Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm, altitude_error_cm, target_climb_rate_cm_s);

	cnt++;


    // body-frame rate controller is run directly from 100hz loop

    // *** NOT SURE ABOUT THE FOLLOWING FUNCTION ***
    // output pilot's throttle
    //attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

//    // if not armed set throttle to zero and exit immediately
//    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
//        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
//        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
//        return;
//    }
//
//    // apply SIMPLE mode transform to pilot inputs
//    update_simple_mode();
//
//    marker_detected = vision_pose.is_marker_detected();
//    frame_number = vision_pose.get_frame_number();
//
//    float takeoff_climb_rate = 0.0f;
//
//    // initialize vertical speeds and acceleration // Limits to the max velocity and acceleration along z axis
//    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
//    pos_control.set_accel_z(g.pilot_accel_z);
//
//    z = vision_pose.get_z_position();
//
//    // get pilot desired climb rate
//    altitude_error = (pos_control.get_alt_target() - z);
//    target_climb_rate = 2.0*altitude_error;
//
//    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
//
//    // call position controller
//    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
//    pos_control.update_z_controller();
//
//    if(cnt%4==0)
//    Log_Write_VisionPose_AH(marker_detected, frame_number, z, altitude_error, target_climb_rate);
//
//	cnt++;

}
