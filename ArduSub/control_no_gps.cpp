#include "Sub.h"


/*
 * this code wants to execute a control without gps
 */

// init control no_gps controller
bool Sub::no_gps_init()
{
    //safe mechanism, have at least some sensor data, we need a barometer
    if(!control_check_barometer()) {
        return false;
    }

    // initialise waypoint controller, 
    // this will init vertical maximun speeds and accels
    wp_nav.wp_and_spline_init();


    // initiate an empty destination objective
    Vector3f stopping_point;
    wp_nav.get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav.set_wp_destination(stopping_point, false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));

    return true;
}

// no_gps_run - runs the controllers
// should be called at 100hz or more
void Sub::no_gps_run()
{
    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        wp_nav.wp_and_spline_init();
        //pos_control.relax_z_controller(motors.get_throttle_hover());
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);


    // Send pilot input to forward/lateral outputs
    motors.set_lateral(channel_lateral->norm_input());
    motors.set_forward(channel_forward->norm_input());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control.update_z_controller();

    attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
}
