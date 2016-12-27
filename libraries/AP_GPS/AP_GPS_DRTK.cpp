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

//  Novatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_DRTK.h"
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

#define NOVA_DEBUGGING 0

#if NOVA_DEBUGGING
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_DRTK::AP_GPS_DRTK(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _buf{0}
{
    const char *init_str = _initialisation_blob[0];
    const char *init_str1 = _initialisation_blob[1];

    port->write((const uint8_t*)init_str, strlen(init_str));
    port->write((const uint8_t*)init_str1, strlen(init_str1));
}

/*
  detect a BDNST GPS. Adds one byte, and returns true if the stream
  matches a BDNST D-RTK --- reserve
 */
//bool AP_GPS_DRTK::_detect(struct DRTK_detect_state &state, uint8_t data)
//{
//reset:
//    switch (state.state) {
//        case DRTK_detect_state::NONE:
//        case DRTK_detect_state::START_SEQ:
//        case DRTK_detect_state::END_SEQ:
//        default:
//            break;
//    }
//    return false;
//}

// Process all bytes available from the stream
//
bool AP_GPS_DRTK::read(void)
{
    bool ret = false;
    int read_len = port->available();

    for (int i=0; i<read_len; i++) {
        _buf[_buf_offset+i] = port->read();
    }

    _buf_offset += read_len;
    int8_t offset = find_char(_buf, START_CHARACTER);
    if (-1 != offset) {
        int end_offset = find_char(_buf, END_CHARACTER);
        if (end_offset != -1) {
            // do nothing if packet isn't complete
            int data_len = find_char(_buf+offset+1, SEPARATOR);
            int64_t start_seq = 0;
            for (int j=1; j<=data_len; j++) {
                start_seq = start_seq << 8;
                start_seq += _buf[offset+j];
            }

            // need to add when get start_seq
            offset += (data_len + 1);

            switch (start_seq) {
                case START_SEQ_PSAT:
                    data_len = find_char(_buf+offset+1, SEPARATOR);
                    int64_t msg_type = 0;

                    for (int j=1; j<=data_len; j++) {
                        msg_type = msg_type << 8;
                        msg_type += _buf[offset+j];
                    }

                    // need to add when get start_seq
                    offset += (data_len + 1);

                    switch (msg_type) {
                        case MESSAGE_TYPE_FVI:
                            get_reality_data(fvi_msg.utc_time, _buf, offset);
                            get_reality_data(fvi_msg.lat, _buf, offset);
                            get_reality_data(fvi_msg.lng, _buf, offset);
                            get_reality_data(fvi_msg.hgt, _buf, offset);
                            get_reality_data(fvi_msg.latsdev, _buf, offset);
                            get_reality_data(fvi_msg.lngsdev, _buf, offset);
                            get_reality_data(fvi_msg.hgtsdev, _buf, offset);
                            get_reality_data(fvi_msg.heading, _buf, offset);
                            get_reality_data(fvi_msg.headingsdev, _buf, offset);
                            get_reality_data(fvi_msg.pitch, _buf, offset);
                            get_reality_data(fvi_msg.pitchsdev, _buf, offset);
                            get_reality_data(fvi_msg.roll, _buf, offset);
                            get_reality_data(fvi_msg.rollsdev, _buf, offset);
                            get_reality_data(fvi_msg.n_vel, _buf, offset);
                            get_reality_data(fvi_msg.e_vel, _buf, offset);
                            get_reality_data(fvi_msg.u_vel, _buf, offset);
                            get_reality_data(fvi_msg.ground_speed, _buf, offset);

                            int8_t current_offset = MASTER_STAR_OFFSET - GROUND_SPEED_OFFSET;
                            get_reality_data(fvi_msg.master_antenna_star, _buf, offset, current_offset);
                            get_reality_data(fvi_msg.sub_antenna_star, _buf, offset);
                            get_reality_data(fvi_msg.position_status, _buf, offset);
                            get_reality_data(fvi_msg.heading_status, _buf, offset);
                            get_reality_data(fvi_msg.baseline, _buf, offset);

                            break;
                        // reserve
                    }

                    // ending code
                    end_offset += 2;
                    data_len = _buf_offset-end_offset-1;
                    for (int i=0; i<data_len; i++) {
                        _buf[i] = _buf[end_offset+i+1];
                    }
                    memset(_buf+data_len, 0, (BUF_SIZE - data_len) * sizeof(char));
                    _buf_offset = data_len;

                    process_message();

                    break;
            }
        }
    } else if (_buf_offset >= BUF_SIZE){
        memset(_buf, 0, BUF_SIZE * sizeof(char));
        _buf_offset = 0;
    }

    return ret;
}

void AP_GPS_DRTK::process_message(void)
{
    state.time_week = 0;
    state.time_week_ms = (uint32_t) fvi_msg.utc_time * 1000;
    uint8_t HH = state.time_week_ms / 10000000;
    uint8_t MM = (state.time_week_ms - HH) / 100000;
    uint8_t SS = (state.time_week_ms - HH - MM) / 1000;
    state.time_week_ms = ((HH * 60 + MM) * 60 + SS) * 1000 + (state.time_week_ms - HH - MM - SS);
    state.last_gps_time_ms = state.time_week_ms;

    state.location.lat = (int32_t) (fvi_msg.lat*1e7);
    state.location.lng = (int32_t) (fvi_msg.lng*1e7);
    state.location.alt = (int32_t) (fvi_msg.hgt*1e2);

    state.num_sats = (fvi_msg.master_antenna_star + fvi_msg.sub_antenna_star) / 2;

    state.horizontal_accuracy = (float) ((fvi_msg.latsdev + fvi_msg.lngsdev)/2);
    state.vertical_accuracy = (float) fvi_msg.hgtsdev;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;

    switch ((uint8_t)fvi_msg.position_status)
    {
        case 1:
            state.status = AP_GPS::GPS_OK_FIX_3D;
            break;
        case 2:
            state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
            break;
        case 4:
        case 5:
            state.status = AP_GPS::GPS_OK_FIX_3D_RTK;
            break;
        case 0: // NONE
        default:
            state.status = AP_GPS::NO_FIX;
            break;
    }

    state.ground_speed = (float) fvi_msg.ground_speed;
    state.velocity.x = (float) fvi_msg.n_vel;
    state.velocity.y = (float) fvi_msg.e_vel;
    state.velocity.z = -(float) fvi_msg.u_vel;
    state.have_vertical_velocity = true;

    state.baseline_cm = fvi_msg.baseline * 100;

    if ((uint8_t)fvi_msg.heading_status) {
        state.have_heading_accuracy = true;
        state.heading = fvi_msg.heading * 100;
    } else {
        state.have_heading_accuracy = false;
    }
}

void AP_GPS_DRTK::inject_data(uint8_t *data, uint8_t len)
{
    if (port->txspace() > len) {
        last_injected_data_ms = AP_HAL::millis();
        port->write(data, len);
    } else {
        Debug("NOVA: Not enough TXSPACE");
    }
}

inline int8_t AP_GPS_DRTK::find_char(char *_buf_, char _c)
{
    int data_len = 0;
    char *p = _buf_;

    while (*p) {
        if (*p == _c) {
            return data_len;
        }
        data_len++;
        p++;
    }
    return -1;
}

inline void AP_GPS_DRTK::HexToFloat(char *_buf_, int _len, double &_data)
{
    double tmp = 0.0f;
    bool reverse = false;
    bool negative = false;

    int j = 0;
    if (*(_buf_+j) == MINUS) {
        negative = true;
        j++;
    }
    for (; j<_len; j++) {
        if (*(_buf_+j) == RADIX_POINT){
            reverse = true;
            _data = tmp;
            tmp = 0.1f;
            continue;
        }
        if (reverse) {
            _data += ((double)(*(_buf_ + j) - 48) * tmp);
            if (tmp == 0.0000001f) {
                break;
            }
            tmp *= 0.1f;
        } else {
            tmp *= 10;
            tmp += (*(_buf_ + j) - 48);
        }
    }
    if (!reverse) {
        _data = tmp;
    } else if (negative) {
        _data *= -1;
    }
}

inline void AP_GPS_DRTK::get_reality_data(double &_data, char *_buf_, int8_t &_index)
{
    int data_len = find_char(_buf_+_index+1, SEPARATOR);
    HexToFloat(_buf_+_index+1, data_len, _data);
    _index += (data_len + 1);
}

inline void AP_GPS_DRTK::get_reality_data(double &_data, char *_buf_, int8_t &_index, int8_t offset)
{
    int data_len;

    for (int j=0; j<offset; j++) {
        data_len = find_char(_buf_+_index+1, SEPARATOR);
        if (j == (offset - 1)) {
            HexToFloat(_buf_+_index+1, data_len, _data);
        }
        _index += (data_len + 1);
    }
}