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
    // @Units: (L/Min
    // @User: Standard
    AP_GROUPINFO("_COF", 0, Flowmeter, _coefficient, 16),

    // @Param: _MIN_RATE
    // @DisplayName: Flowmeter minimum flowrate
    // @Description: Minimum flowrate in L/Min that Flowmeter can reliably read
    // @Units: centimeters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN_RATE",  1, Flowmeter, _min_flowrate, 0.0f),

    // @Param: _MAX_RATE
    // @DisplayName: Flowmeter maximum flowrate
    // @Description: Maximum flowrate in L/Min that Flowmeter can reliably read
    // @Units: centimeters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_RATE",  2, Flowmeter, _max_flowrate, 10.0f),

    AP_GROUPEND
};

Flowmeter::Flowmeter(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager),
    _last_timestamp(0),
    _last_pulse_time_ms(0),
    _disable_time_ms(0),
    _good_sample_count(0),
    _last_sample_flowrate(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state
    memset(state,0,sizeof(state));
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

    struct pwm_input_s pwm;
    float sum_flowrate = 0;
    uint16_t count = 0;
    uint32_t now = AP_HAL::millis();

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {

        _last_pulse_time_ms = now;

        float pluse_rate = 1.0f / (float(pwm.period) * 1e-6f);
        /* F(Hz) = [FLOWMETER_CONVERSION_COEFFICIENT * Q]. err = 10%. F = pulse_rate, Q = flowrate */
        float flowrate = pluse_rate / _coefficient;

        float flowrate_delta = fabsf(flowrate - _last_sample_flowrate);
        _last_sample_flowrate = flowrate;

        if (flowrate_delta > VALID_FLOWRATE_DELTA) {
            // varying by more than 1L/min in a single sample,
            _good_sample_count = 0;
            continue;
        }

        if (_good_sample_count > 1) {
            count++;
            sum_flowrate += flowrate;
            _last_timestamp = pwm.timestamp;
        } else {
            _good_sample_count++;
        }
    }

    // if we have not taken a reading in the last 0.2s set status to No Data
    if (AP_HAL::micros64() - _last_timestamp >= 200000) {
        set_status(Flowmeter::Flowmeter_NoData);
    }

    /* if we haven't seen any pulses for 0.5s then the sensor is
       probably dead. Try resetting it. Tests show the sensor takes
       about 0.2s to boot, so 500ms offers some safety margin
    */
    if (now - _last_pulse_time_ms > 500U && _disable_time_ms == 0) {
        ioctl(_fd, SENSORIOCRESET, 0);
        _last_pulse_time_ms = now;
    }

    if (count != 0) {
        state.flowrate = sum_flowrate / count;

        // update range_valid state based on distance measured
        update_status();
        if (state.flowrate > FLOWMETER_ENABLE_FLOWRATE) {
            enable();
        }
    }
}

// true if sensor is returning data
bool Flowmeter::has_data() const
{
    return ((state.status != Flowmeter_NotConnected) && (state.status != Flowmeter_NoData));
}

void Flowmeter::set_status(Flowmeter::Flowmeter_Status status)
{
    state.status = status;

    // update valid count
    if (status == Flowmeter::Flowmeter_Good) {
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
