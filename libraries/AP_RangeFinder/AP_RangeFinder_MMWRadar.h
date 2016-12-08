// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <GCS_MAVLink/GCS.h>

#define MMWRADAR_UPDATE_IN_HZ 50.0f
#define MMWRADAR_DATA_BUFFER_NUM 5  // max target can achieve 255, but now only can check 1
#define MMWRADAR_DATA_BUFFER_SIZE 10
#define MMWRADAR_UART_BUFFER_SIZE 420

#define MMWRADAR_START_SEQUENCE_H      0xAA
#define MMWRADAR_START_SEQUENCE_L      0xAA
#define MMWRADAR_END_SEQUENCE_H        0x55
#define MMWRADAR_END_SEQUENCE_L        0x55

#define MMWRADAR_DATA_REL_OFFSET        1
#define MMWRADAR_END_REL_OFFSET        11

/* Message ID */
#define MMWRADAR_MSGID_CONFIGURATION_H 0x02
#define MMWRADAR_MSGID_CONFIGURATION_L 0x00
#define MMWRADAR_MSGID_SENSOR_BACK    0x400
#define MMWRADAR_MSGID_SENSOR_STATUS  0x60A
#define MMWRADAR_MSGID_TARGET_STATUS  0x70B
#define MMWRADAR_MSGID_TARGET_INFO    0x70C
// only for sp25
#define MMWRADAR_MSGID_SPEED_INFO     0x300
#define MMWRADAR_MSGID_YAWRATE_INFO   0x301
#define MMWRADAR_MSGID_VERSION        0x800

/* SENSOR CONFIGURATION */
// DataType 0 ~ 6 Bit
#define MMWRADAR_DATATYPE_SENSOR_ID         0x1
#define MMWRADAR_DATATYPE_SENSOR_VERSION    0x2
#define MMWRADAR_DATATYPE_SENSOR_SWITCH     0x3
#define MMWRADAR_DATATYPE_FILTER            0x4
#define MMWRADAR_DATATYPE_INTEST            0x7e
#define MMWRADAR_DATATYPE_SAVE_PARAMETER    0x7f
// R/W 7 Bit
#define MMWRADAR_CON_READ   0
#define MMWRADAR_CON_WRITE  1
// Parameter 8 ~ 31, Reserve 32 ~ 63, only support read sensor version.

/* Sensor Back after config */
// Result 7 Bit
#define MMWRADAR_READ_STATUS_OFFSET 2
#define MMWRADAR_READ_STATUS_BIT 0x80
#define MMWRADAR_MASTER_VERSION_OFFSET 3
#define MMWRADAR_SECOND_VERSION_OFFSET 4
#define MMWRADAR_STEP_VERSION_OFFSET 5

/* SYSTEM STATUS */
#define MMWRADAR_SENSOR_STATUS_ROLLCOUNT_OFFSET 3
#define MMWRADAR_SENSOR_STATUS_ROLLCOUNT_BIT    0x03

/* TARGET STATUS */
#define MMWRADAR_TARGET_STATUS_TARGETNUM_OFFSET 2
#define MMWRADAR_TARGET_STATUS_ROLLCOUNT_OFFSET 3
#define MMWRADAR_TARGET_STATUS_ROLLCOUNT_BIT    0x03

/* TARGET INFO */
#define MMWRADAR_TARGET_INFO_TARGETID_OFFSET    2
#define MMWRADAR_TARGET_INFO_RCS_OFFSET         3
#define MMWRADAR_TARGET_INFO_RANGE_H_OFFSET     4
#define MMWRADAR_TARGET_INFO_RANGE_L_OFFSET     5
#define MMWRADAR_TARGET_INFO_VEL_H_OFFSET       7
#define MMWRADAR_TARGET_INFO_VEL_H_BIT          0x07
#define MMWRADAR_TARGET_INFO_VEL_L_OFFSET       8
#define MMWRADAR_TARGET_INFO_SNR_OFFSET         9

class AP_RangeFinder_MMWRadar : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MMWRadar(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state, AP_SerialManager &serial_manager);

    // update state
    void update(void);

    inline char get_data_from_buf(int packet, int offset, int bit) {
        return (char)(_valid_data_buf[packet][offset] & bit);
    }

    inline char get_data_from_buf(int packet, int offset) {
        return _valid_data_buf[packet][offset];
    }

    struct Target_Info_Type {
        uint8_t  index;
        uint16_t range_cm;
        uint16_t rcs_cm;
        uint16_t vel_m;
        uint16_t snr;
    };

    struct Target_Status_Type {
        uint8_t  target_num;
        uint8_t  rollcount;
    };

    enum Packet_Form {
        None = 0,
        Sensor_Back = 1,
        Sensor_Status = 2,
        Target_Status = 4,
        Target_Info = 8,
        Status_error = 16
    };

private:
    // get a reading
    bool get_reading();

    // get sensor version
    bool get_sensor_version(uint32_t &sensor_version);

    // parse
    uint16_t parse();

    // receive packet from uart
    bool recv_packet();

    // reset all parameter
    void reset_param();

    AP_HAL::UARTDriver *uart = nullptr;

    char _valid_data_buf[MMWRADAR_DATA_BUFFER_NUM][MMWRADAR_DATA_BUFFER_SIZE];
    char _data_buf[MMWRADAR_UART_BUFFER_SIZE];
    int  _data_buf_offset;
    uint8_t _valid_data_packet;
    Target_Status_Type _target_status;
    Target_Info_Type   _target_info;
    char     _master_version;
    char     _second_version;
    char     _step_version;
    uint32_t    _last_reading_ms;
    uint16_t  _packet_form;
};


