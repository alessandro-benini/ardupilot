/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
extern const AP_HAL::HAL& hal;

static bool land_with_vision;

static uint32_t land_start_time;
static bool land_pause;

bool Copter::vision_land_init(bool ignore_checks)
{
    // set to position control mode
	// TODO: Double check if this command means that the pilot will take care of the landing manually
    guided_mode = Guided_WP;

	// FROM CONTROL_GUIDED
    // initialise wpnav destination

    // current_loc is initialized with the output of the navigation Kalman Filter
    // This means, when I switch to the autonomous landing mode, the quadrotor read its actual position
    // and defines it as the starting point of the autonomous landing procedure

    // TODO: Uncomment the following line of code, when writing the controller
    // Location_Class target_loc = current_loc;

    // this instruction could not be necessary
    // target_loc.set_alt_cm(final_alt_above_home, Location_Class::ALT_FRAME_ABOVE_HOME);

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

// YAW seems to be set by the pilot, while ROll and PITCH seems to be set by the software.

/**
 * The landing phase will be made in two steps:
 * 1) alignment of the quadrotor with the platform (x,y,z,yaw);
 * 2) landing decreasing the altitude
 * Since the quadrotor can drift during the landing phase, the two steps must be executed consecutively at each step.
 */
void Copter::vision_land_run()
{

	//float x = vision_pose.get_x_position();
	//hal.console->printf("Received message\n");
	// Here it comes the control system for landing the Helicopter.

	// Target values for yaw, pitch, roll
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs --> THIS COULD NOT BE NECESSARY SINCE THE COMMUNICATION WITH THE RADIO COULD FAIL
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // THIS IS NECESSARY if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    // target roll and pitch should be put to zero during the landing phase.
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // consider to use an hysteresis counter to calculate when to switch from one controller to the another.
    int hysteresis_counter = 0;

    if(vision_pose.is_marker_detected())
    {
    	// 1. Get the current position of the UAV w.r.t. to the marker (x,y,z) from the vision_pose packet
    	// 2. Calculate the distance from home (home will be always (0,0,0) (center of the marker)
    	// 3. calculate control action
    	// 4. apply control action
    }
    else
    {
    	// 1. Get the current position of the UAV w.r.t. to the marker (lat,long,alt) from the GPS
    	// 2. Calculate the distance from home (home will be the GPS position of the marker, with a fix altitude of 2 m)
    	// 3. calculate control action
    	// 4. apply control action
    }







    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

}
