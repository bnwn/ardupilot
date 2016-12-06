// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <GCS_MAVLink/GCS.h>

#define MMWRADAR_UPDATE_IN_HZ 50.0f
#define MMWRADAR_DATA_BUFFER_SIZE 10

#define MMWRADAR_START_SEQUENCE_H      0xAA
#define MMWRADAR_START_SEQUENCE_L      0xAA
#define MMWRADAR_END_SEQUENCE_H        0x55
#define MMWRADAR_END_SEQUENCE_L        0x55

/* Message ID */
#define MMWRADAR_MSGID_CONFIGURATION_H 0x02
#define MMWRADAR_MSGID_SENSOR_BACK_H   0x04
#define MMWRADAR_MSGID_SENSOR_STATUS_H 0x06
#define MMWRADAR_MSGID_TARGET_STATUS_H 0x07
#define MMWRADAR_MSGID_TARGET_INFO_H   0x07
#define MMWRADAR_MSGID_CONFIGURATION_L 0x00
#define MMWRADAR_MSGID_SENSOR_BACK_L   0x00
#define MMWRADAR_MSGID_SENSOR_STATUS_L 0x0A
#define MMWRADAR_MSGID_TARGET_STATUS_L 0x0B
#define MMWRADAR_MSGID_TARGET_INFO_L   0x0C
// only for sp25
#define MMWRADAR_MSGID_SPEED_INFO_H    0x03
#define MMWRADAR_MSGID_YAWRATE_INFO_H  0x03
#define MMWRADAR_MSGID_VERSION_H       0x08
#define MMWRADAR_MSGID_SPEED_INFO_L    0x00
#define MMWRADAR_MSGID_YAWRATE_INFO_L  0x01
#define MMWRADAR_MSGID_VERSION_L       0x00

/* SENSOR CONFIGURATION */
// DataType 0 ~ 6 Bit
#define MMWRADAR_DATATYPE_SENSOR_ID         0x01
#define MMWRADAR_DATATYPE_SENSOR_VERSION    0x02
#define MMWRADAR_DATATYPE_SENSOR_SWITCH     0x03
#define MMWRADAR_DATATYPE_FILTER            0x04
#define MMWRADAR_DATATYPE_INTEST            0x7e
#define MMWRADAR_DATATYPE_SAVE_PARAMETER    0x7f
// R/W 7 Bit
#define MMWRADAR_CON_READ   0
#define MMWRADAR_CON_WRITE  1
// Parameter 8 ~ 31, Reserve 32 ~ 63, only support read sensor version.

/* Sensor Back after config */
// Result 7 Bit
#define MMWRADAR_RETURN_FAILURE 0
#define MMWRADAR_RETURN_SUCCESS 1

/* SYSTEM STATUS */
#define MMWRADAR_SENSOR_STATUS_ROLLCOUNT_OFFSET 1
#define MMWRADAR_SENSOR_STATUS_ROLLCOUNT_BIT    0x03

/* TARGET STATUS */
#define MMWRADAR_TARGET_STATUS_TARGETNUM_OFFSET 0
#define MMWRADAR_TARGET_STATUS_ROLLCOUNT_OFFSET 1
#define MMWRADAR_TARGET_STATUS_ROLLCOUNT_BIT    0x03

/* TARGET INFO */
#define MMWRADAR_TARGET_INFO_TARGETID_OFFSET    0
#define MMWRADAR_TARGET_INFO_RCS_OFFSET         1
#define MMWRADAR_TARGET_INFO_RANGE_H_OFFSET     2
#define MMWRADAR_TARGET_INFO_RANGE_L_OFFSET     3
#define MMWRADAR_TARGET_INFO_VEL_H_OFFSET       5
#define MMWRADAR_TARGET_INFO_VEL_H_BIT          0x07
#define MMWRADAR_TARGET_INFO_VEL_L_OFFSET       6
#define MMWRADAR_TARGET_INFO_SNR_OFFSET         7

class AP_RangeFinder_MMWRadar : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MMWRadar(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state, AP_SerialManager &serial_manager);

    // update state
    void update(void);

    inline char get_data_from_buf(uint8_t offset, uint8_t bit) {
        return (_valid_data_buf[offset] & bit);
    }

    inline char get_data_from_buf(uint8_t offset) {
        return _valid_data_buf[offset];
    }

    struct Target_Info {
        uint8_t  index;
        float    range_cm;
        float    rcs;
        float    vel_m;
        float    snr;
    };

    struct Target_Status {
        uint8_t  target_num;
        uint8_t  rollcount;
    };

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    // get sensor version
    bool get_sensor_version(uint32_t &sensor_version);

    AP_HAL::UARTDriver *uart = nullptr;

    char _valid_data_buf[MMWRADAR_DATA_BUFFER_SIZE];
    Target_Status _target_status;
    Target_Info   _target_info;
};


