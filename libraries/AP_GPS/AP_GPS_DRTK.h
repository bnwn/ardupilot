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

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#define BUF_SIZE 500
#define MESSAGE_TYPE_FVI 0x465649
#define START_CHARACTER 0x24
#define START_SEQ_GNGGA 0x474E4747
#define START_SEQ_PSAT 0x50534154
#define SEPARATOR 0x2C
#define END_CHARACTER 0x2A
#define RADIX_POINT 0x2E
#define MINUS 0x2D

// use to fvi message
#define FVI_UTC_TIME_OFFSET 1
#define FVI_LATTITUDE_OFFSET 2
#define FVI_LONGITUDE_OFFSET 3
#define FVI_ELEVATION_OFFSET 4
#define FVI_LATTITUDE_VARIANCE_OFFSET 5
#define FVI_LONGITUDE_VARIANCE_OFFSET 6
#define FVI_ELEVATION_VARIANCE_OFFSET 7
#define FVI_HEADING_OFFSET 8
#define FVI_HEADING_VARIANCE_OFFSET 9
#define FVI_PITCH_OFFSET 10
#define FVI_PITCH_VARIANCE_OFFSET 11
#define FVI_ROLL_OFFSET 12
#define FVI_ROLL_VARIANCE_OFFSET 13
#define FVI_EAST_SPEED_OFFSET 14
#define FVI_NORTH_SPEED_OFFSET 15
#define FVI_UP_SPEED_OFFSET 16
#define FVI_GROUND_SPEED_OFFSET 17
#define FVI_EAST_COORDINATE_OFFSET 18
#define FVI_NORTH_COORDINATE_OFFSET 19
#define FVI_UP_COORDINATE_OFFSET 20
#define FVI_MASTER_STAR_OFFSET 24
#define FVI_SUB_STAR_OFFSET 25
#define FVI_POSITION_STATUS_OFFSET 25
#define FVI_HEADING_STATUS_OFFSET 27
#define FVI_BASELINE_LENGTH 28

// use to gngga message
#define GGA_PDOP_OFFSET 8
#define GGA_ELEVATION_OFFSET 9

class AP_GPS_DRTK : public AP_GPS_Backend
{
public:
    AP_GPS_DRTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // --reserve
//    bool _detect(struct DRTK_detect_state &state, uint8_t data);
    // Methods
    bool read();

    void inject_data(uint8_t *data, uint8_t len);

    enum packet_type {
        NONE = 0,
        GNGGA = 1,
        PSAT_FVI = 2,
    };

private:

    void process_message(enum packet_type _packet);

    inline void find_char(char *_buf_, char _c, int16_t &_len);
    inline void HexToFloat(char *_buf_, int16_t _len, double &_data);
    inline void get_reality_data(double &_data, char *_buf_, int16_t &_index);
    inline void get_reality_data(double &_data, char *_buf_, int16_t &_index, int8_t offset);

    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    const char* _initialisation_blob[2] = {
        "log gga ontime 0.1\r\nlog fvi ontime 0.1\r\n", // get gngga and fvi
        "log fvi ontime 0.1\r\n", // get fvi
    };

    uint32_t last_hdop = 999;
    uint32_t last_injected_data_ms = 0;
    char _buf[BUF_SIZE];
    uint16_t _buf_offset = 0;

    struct fvi
    {
        double utc_time;
        double lat;
        double lng;
        double hgt;
        double latsdev;
        double lngsdev;
        double hgtsdev;
        double heading;
        double headingsdev;
        double pitch;
        double pitchsdev;
        double roll;
        double rollsdev;
        double n_vel;
        double e_vel;
        double u_vel;    // neu vel (m/s)
        double ground_speed;
        double n_cooradinate;
        double e_cooradinate;
        double u_cooradinate;
        double master_antenna_star;
        double sub_antenna_star;
        double position_status;
        double heading_status;
        double baseline;
    } fvi_msg;

    struct gga {
        double pdop;
        double elevation;
    } gga_msg;

};
