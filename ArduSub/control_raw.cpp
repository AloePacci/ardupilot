#include "Sub.h"

// raw_init - initialise manual controller
bool Sub::raw_init()
{
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}

// raw_run - runs the manual (bypass) controller
// should be called at 100hz or more
void Sub::raw_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    for(int motor_number=0;motor_number<8;motor_number++){
        motors.rc_write(motor_number, RC_Channels::rc_channel(motor_number)->get_radio_in());
    }
}
