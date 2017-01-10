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

#define BUF_SIZE 400
#define MESSAGE_TYPE_FVI 0x465649
#define START_CHARACTER 0x24
#define START_SEQ_PSAT 0x50534154
#define SEPARATOR 0x2C
#define END_CHARACTER 0x2A
#define RADIX_POINT 0x2E
#define MINUS 0x2D

#define UTC_TIME_OFFSET 1
#define LATTITUDE_OFFSET 2
#define LONGITUDE_OFFSET 3
#define ELEVATION_OFFSET 4
#define LATTITUDE_VARIANCE_OFFSET 5
#define LONGITUDE_VARIANCE_OFFSET 6
#define ELEVATION_VARIANCE_OFFSET 7
#define HEADING_OFFSET 8
#define HEADING_VARIANCE_OFFSET 9
#define PITCH_OFFSET 10
#define PITCH_VARIANCE_OFFSET 11
#define ROLL_OFFSET 12
#define ROLL_VARIANCE_OFFSET 13
#define EAST_SPEED_OFFSET 14
#define NORTH_SPEED_OFFSET 15
#define UP_SPEED_OFFSET 16
#define GROUND_SPEED_OFFSET 17
#define EAST_COORDINATE_OFFSET 18
#define NORTH_COORDINATE_OFFSET 19
#define UP_COORDINATE_OFFSET 20
#define MASTER_STAR_OFFSET 24
#define SUB_STAR_OFFSET 25
#define POSITION_STATUS_OFFSET 26
#define HEADING_STATUS_OFFSET 27
#define BASELINE_LENGTH 28

class AP_GPS_DRTK : public AP_GPS_Backend
{
public:
    AP_GPS_DRTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    // --reserve
//    bool _detect(struct DRTK_detect_state &state, uint8_t data);
    // Methods
    bool read();

    void inject_data(uint8_t *data, uint8_t len);

private:

    void process_message();

    inline void find_char(char *_buf_, char _c, int16_t &_len);
    inline void HexToFloat(char *_buf_, int16_t _len, double &_data);
    inline void get_reality_data(double &_data, char *_buf_, int16_t &_index);
    inline void get_reality_data(double &_data, char *_buf_, int16_t &_index, int8_t offset);

    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    const char* _initialisation_blob[2] = {
        "unlogall\r\n", // cleanup enviroment
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

};
