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

int counter_for_landing = 0;
int start_landing = 16000; // after 5 seconds of hovering, the landing procedure starts
float th_error = 150.0; // 15 cms max error

float z_setpoint = 150.0;

Buzzer _buzzer;

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
    pos_control.set_alt_target(z_setpoint);
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

    float max_angle = 175.0f; // 3 degrees

	int frame_number;
	int marker_detected;
	float yaw_rad = 0.0f;
	float target_roll = 0.0f, target_pitch = 0.0f;

	Vector3f position, velocity;
	Vector3f position_NED, velocity_NED;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    current_state = vision_pose.getState();

    frame_number = current_state->frame_number;
    marker_detected = current_state->marker_detected;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

	set_auto_yaw_mode(AUTO_YAW_HOLD);

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // hal.console->printf("Current State: %d %f %f %f %f \n", marker_detected, current_state->position.x, current_state->position.y, current_state->velocity.x, current_state->velocity.y);

    // posX_cm, posY_cm, posZ_cm are expressed in marker reference frame


	// If I detected the marker I updated the velocity
    if(marker_detected == 1)
    {
    	position = current_state->position;
    	velocity = current_state->velocity;
    	yaw_rad = ((float)ahrs.yaw_sensor/100.0)*3.1415/180.0f;
    // Otherwise, for security, I set the velocity to zero.
    }
    else
    	velocity = Vector3f(0.0f,0.0f,0.0f);

    position_NED.x = position.x*cos(yaw_rad)-position.y*sin(yaw_rad);
    position_NED.y = position.x*sin(yaw_rad)+position.y*cos(yaw_rad);
    position_NED.z = position.z;

    velocity_NED.x = velocity.x*cos(yaw_rad)-velocity.y*sin(yaw_rad);
    velocity_NED.y = velocity.x*sin(yaw_rad)+velocity.y*cos(yaw_rad);
    velocity_NED.z = velocity.z;

    float dt = pos_control.time_since_last_xy_update();

    if (dt >= pos_control.get_dt_xy()) {
    	pos_control._update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, 1.0, false, marker_detected, position_NED, velocity_NED);
    }

//    if(marker_detected == 1)
//    {
        target_roll  = pos_control.get_roll();
        target_pitch = pos_control.get_pitch();
//    }
//    else
//    {
//        target_roll  = 0.0f;
//        target_pitch = 0.0f;
//    }

    // hal.console->printf("Roll_Pitch: %f %f %f %f %f %f\n",position.x, position.y, velocity.x, velocity.y, target_roll,target_pitch);

//    if(target_roll > max_angle)
//    	target_roll = max_angle;
//    if(target_roll < -max_angle)
//    	target_roll = -max_angle;
//
//    if(target_pitch > max_angle)
//    	target_pitch = max_angle;
//    if(target_pitch < -max_angle)
//    	target_pitch  = -max_angle;

    if(marker_detected == 1 && position.z != 0)
    	pos_control._update_z_controller(marker_detected, -position.z);
    else
    	pos_control.update_z_controller();

    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, 0.0, get_smoothing_gain());

    if(counter_for_landing < start_landing)
    {
        if(marker_detected == 1 && fabsf(position_NED.x) < th_error && fabsf(position_NED.y) < th_error)
        {
        	// hal.console->printf("PREPARING FOR LANDING %d\n",counter_for_landing);
        	counter_for_landing += 1;
        }
        else
        {
        	counter_for_landing = 0;
        	// hal.console->printf("NO LANDING %d\n",counter_for_landing);
        }
    }
    else // if(counter_for_landing > start_landing) //  && (counter_for_landing % 20 == 0))
    {
    	_buzzer.AL();
    	counter_for_landing += 1;
    	if(fabsf(position_NED.z) > 50.0)
    	{
        	// hal.console->printf("LANDING %f\n",z_setpoint);
        	if(z_setpoint > 0 && counter_for_landing % 20 == 0)
        		z_setpoint -= 0.5;
        	pos_control.set_alt_target(z_setpoint);
    	}
    	else
    		pos_control.set_alt_target(0.0f);
    }

	if(cnt%2==0)
	{
		Log_Write_VisionPose_XY(marker_detected, frame_number, position_NED, velocity_NED, target_roll, target_pitch, yaw_rad);
		// Log_Write_VisionPose_AH(marker_detected, frame_number, posZ_cm);
	}
	cnt++;
}
