#include "Copter.h"

#define PUMP_CONTROL_CHANNEL 5
#define PUMP_OUTPUT_CHANNEL 3

void Copter::init_farming()
{
    init_flowmeter();
    radio_to_pump_output();
}

// set output throngh radio channel
void Copter::radio_to_pump_output()
{
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_sprayer_pump, CH_8);
}

void Copter::pesticide_remaining_check()
{
    if (!get_pesticide_remaining()
            && (control_mode == POINT_ATOB)
            && flowmeter.farming_state()
            && !ap.land_complete
//            && (SRV_Channels::get_servo(SRV_Channel::k_sprayer_pump) > (sprayer.get_pump_rate() * 100))) {
            ) {
        // only return home when pesticide is empty
        set_mode(RTL, MODE_REASON_PESTICIDE_EMPTY);
    }
//    static int i = 0;
//    if (!get_pesticide_remaining()) {
//        i++;
//        if (i > 10) {
//            printf("pesticide empty RTL!\n");
//            i=0;
//        }
//    } else {
//        printf("flowrate_rel: %d \n  pluse rate: %d \n", flowmeter_state.flowrate, flowmeter.pluse_rate());
//    }
}

void Copter::farming_mode_handle(void)
{
    pesticide_remaining_check();
}
