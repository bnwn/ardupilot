#include "Copter.h"

#define PUMP_CONTROL_CHANNEL 5
#define PUMP_OUTPUT_CHANNEL 3

void Copter::pump_output_init()
{

}

// set output throngh radio channel
void Copter::radio_to_pump_output()
{

//    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_motor_tilt, v);
}

void Copter::pesticide_remaining_check()
{
    static bool is_spraying_status = false;

    uint16_t v = hal.rcin->read(PUMP_CONTROL_CHANNEL);
}
