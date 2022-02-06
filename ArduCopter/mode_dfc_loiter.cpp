#include "Copter.h"

#if MODE_LOITER_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeDFCLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd()); //attitude_control->get_althold_lean_angle_max()

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeDFCLoiter::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);


        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);

		//grab loiter_nav thrust vector -> the X/Y go to DFC commands
		// thrust vector is NED
		//grab the nipples and use them for attitude cmd
        /*
        Quaternion attitude_curr;                   // Rotation to current MAV_FRAME_BODY_FRD from MAV_FRAME_LOCAL_NED
        attitude_curr.from_euler(roll, pitch, yaw); // Rotation to MAV_FRAME_LOCAL_NED from current MAV_FRAME_BODY_FRD
        attitude_curr.invert();
        Matrix3f body_ned_m;
        attitude_curr.rotation_matrix(body_ned_m);
        */


        // adjust climb rate using rangefinder
        //target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        //target_climb_rate = get_avoidance_adjusted_climbrate(0);//target_climb_rate

        ///pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate, false);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}


#endif
