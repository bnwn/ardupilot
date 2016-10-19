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
    if (flowmeter.farming_state() && !ap.land_complete && !get_pesticide_remaining()) {
        // pesticide is empty
        set_mode(RTL, MODE_REASON_PESTICIDE_EMPTY);
    }
        /* test
        printf("pesticide empty RTL!\n");
    } else {
        printf("flowrate_rel: %d \n  pluse rate: %d \n", flowmeter_state.flowrate, flowmeter.pluse_rate());
    }*/
}

void Copter::farming_mode_handle(void)
{
    pesticide_remaining_check();
}
