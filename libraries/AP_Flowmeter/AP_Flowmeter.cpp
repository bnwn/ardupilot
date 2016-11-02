// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Flowmeter.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <cmath>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Flowmeter::var_info[] = {

    // @Param: _COF
    // @DisplayName: Flowmeter coefficient
    // @Description: F=coefficient*Q(L/Min)
    // @Units: L/Min
    // @User: Enigma
    AP_GROUPINFO("_COF", 0, Flowmeter, _coefficient, 16),

    // @Param: _MIN_RATE
    // @DisplayName: Flowmeter minimum flowrate
    // @Description: Minimum flowrate in L/Min that Flowmeter can reliably read
    // @Units: L/Min
    // @Increment: 1
    // @User: Enigma
    AP_GROUPINFO("_MIN_RATE",  1, Flowmeter, _min_flowrate, 0.0f),

    // @Param: _MAX_RATE
    // @DisplayName: Flowmeter maximum flowrate
    // @Description: Maximum flowrate in L/Min that Flowmeter can reliably read
    // @Units: L/Min
    // @Increment: 1
    // @User: Enigma
    AP_GROUPINFO("_MAX_RATE",  2, Flowmeter, _max_flowrate, 10.0f),

    // @Param: _FARMING
    // @DisplayName: farming mode switch
    // @Description: control farming mode on/off
    // @Default: 233(on)
    // @User: Enigma
    AP_GROUPINFO("_FARMING",  3, Flowmeter, _farming_mode, 233),

    AP_GROUPEND
};

Flowmeter::Flowmeter(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager),
    _last_timestamp(0),
    _flowrate_array{0}
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state
    memset(&state,0,sizeof(state));
}

Flowmeter::~Flowmeter()
{
    if (_fd != -1) {
        close(_fd);
    }
    set_status(Flowmeter::Flowmeter_NotConnected);
}

bool Flowmeter::init()
{
    _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Unable to open PX4 PWM Flowmeter\n");
        set_status(Flowmeter::Flowmeter_NotConnected);
        return false;
    }

    // keep a queue of 20 samples
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
        hal.console->printf("Failed to setup range finder queue\n");
        set_status(Flowmeter::Flowmeter_NotConnected);
        return false;
    }

    // initialise to connected but no data
    set_status(Flowmeter::Flowmeter_NoData);
    return true;
}

/*
  update Flowmeter state for all instances. This should be called at
  around 10Hz by main loop
 */
void Flowmeter::update(void)
{
    if (_fd == -1) {
        set_status(Flowmeter::Flowmeter_NotConnected);
        return;
    }

    uint32_t now = AP_HAL::millis();
    struct pwm_input_s pwm;
    float sum_flowrate = 0.0f;
    float flowrate_max = 0.0f, flowrate_min = HUGE_VALF;
    float pluse_rate_tmp = 0.0f;
    uint16_t count = 0;
    float sum_pluse_rate = 0.0f;
    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        pluse_rate_tmp = 1.0f / (float(pwm.period) * 1e-6f);
        sum_pluse_rate += pluse_rate_tmp;
        count++;
    }

    if (count > 0) {
        pluse_rate_tmp = sum_pluse_rate / (float)count;
    }

    /* F(Hz) = [FLOWMETER_CONVERSION_COEFFICIENT * Q]. err = 10%. F = pulse_rate, Q = flowrate */
    float flowrate_tmp = pluse_rate_tmp / _coefficient;

    for (int i=1; i<SAMPLE_INSTANCE; i++) {
        _flowrate_array[i-1] = _flowrate_array[i];
    }

    _flowrate_array[SAMPLE_INSTANCE-1] = flowrate_tmp;

    for (int i=0; i<SAMPLE_INSTANCE; i++) {

        if (_flowrate_array[i] > flowrate_max) {
            flowrate_max = _flowrate_array[i];
        }

        if (_flowrate_array[i] < flowrate_min) {
            flowrate_min = _flowrate_array[i];
        }

        sum_flowrate += _flowrate_array[i];
    }

    state.flowrate = (sum_flowrate - flowrate_max - flowrate_min) / (float)(SAMPLE_INSTANCE - 2);
    state.pluse_rate = pluse_rate_tmp;

    // update range_valid state based on distance measured
    update_status();

    // keep a certain extent time to avoid erroneous judgement
    if (state.flowrate > FLOWMETER_ENABLE_FLOWRATE && !state.enabled) {
        if (state.avoid_erroneous_judgement_time == 0) {
            state.avoid_erroneous_judgement_time = now;
        } else if (now - state.avoid_erroneous_judgement_time > PESTICIDE_SPRAYING_ON_TIME_MS) {
            enable();
        }
    } else {
        state.avoid_erroneous_judgement_time = 0;
    }
}

// true if sensor is returning data
bool Flowmeter::has_data() const
{
    return ((state.status != Flowmeter_NotConnected) && (state.status != Flowmeter_NoData));
}

void Flowmeter::set_status(Flowmeter::Flowmeter_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == Flowmeter::Flowmeter_Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

void Flowmeter::update_status(void)
{
    // check distance
    if ((float)state.flowrate > _max_flowrate) {
        set_status(Flowmeter::Flowmeter_OutOfRangeHigh);
    } else if ((float)state.flowrate < _min_flowrate) {
        set_status(Flowmeter::Flowmeter_OutOfRangeLow);
    } else {
        set_status(Flowmeter::Flowmeter_Good);
    }
}
