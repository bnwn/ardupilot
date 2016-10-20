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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define FLOWMETER_ENABLE_FLOWRATE 1.0f
#define VALID_FLOWRATE_DELTA 1.0f
#define SAMPLE_INSTANCE 3

class Flowmeter
{
public:
    Flowmeter(AP_SerialManager &_serial_manager);
    ~Flowmeter();

    enum Flowmeter_Status {
        Flowmeter_NotConnected = 0,
        Flowmeter_NoData,
        Flowmeter_OutOfRangeLow,
        Flowmeter_OutOfRangeHigh,
        Flowmeter_Good
    };

    // The Flowmeter_State structure is filled in by the backend driver
    struct Flowmeter_State {
        float               flowrate; // flowrate
        float               pluse_rate;
        enum Flowmeter_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                enabled;
    };

    // parameters for each instance
    AP_Float _coefficient;
    AP_Float _min_flowrate;
    AP_Float _max_flowrate;
    AP_Int16  _farming_mode;

    static const struct AP_Param::GroupInfo var_info[];

    bool init();

    // update state of all Flowmeters. Should be called at around
    // 10Hz from main loop
    void update(void);

#define _Flowmeter_STATE state

    int16_t flowrate() const {
        return (int16_t)(_Flowmeter_STATE.flowrate * 100);
    }

    int16_t pluse_rate() const {
        return (int16_t)_Flowmeter_STATE.pluse_rate;
    }

    bool farming_state() const {
        return (_farming_mode == 233);
    }

    float max_flowrate() const {
        return _max_flowrate;
    }

    float min_flowrate() const {
        return _min_flowrate;
    }

    // query status
    Flowmeter_Status status() const {
        return state.status;
    }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t range_valid_count() const {
        return _Flowmeter_STATE.range_valid_count;
    }

    void set_status(Flowmeter::Flowmeter_Status _status);

    void update_status(void);

    bool get_state() const {
        return _Flowmeter_STATE.enabled;
    }

    void enable() {
        _Flowmeter_STATE.enabled = true;
    }

    void disable() {
        _Flowmeter_STATE.enabled = false;
    }

private:
    Flowmeter_State state;
    AP_SerialManager &serial_manager;

    int _fd;
    uint64_t _last_timestamp;
    float _flowrate_array[SAMPLE_INSTANCE];
};
