// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_RangeFinder_MMWRadar.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MMWRadar::AP_RangeFinder_MMWRadar(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MMWRadar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_MMWRadar, 0));
    }
}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_MMWRadar::detect(RangeFinder &ranger, uint8_t instance,
                                     RangeFinder::RangeFinder_State &_state, AP_SerialManager &serial_manager)
{
    if (serial_manager.find_serial(AP_SerialManager::SerialProtocol_MMWRadar, 0) != nullptr) {
        AP_RangeFinder_MMWRadar *sensor
            = new AP_RangeFinder_MMWRadar(_ranger, instance, _state, serial_manager);
        uint32_t version;
        if (sensor && sensor->get_sensor_version(version)) {
                return sensor;
        }
    }

    delete sensor;
    return nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_MMWRadar::get_reading()
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the mmw radar
    reset_param();
    if (recv_packet()) {
        uint8_t packet_form = parse();

        if ((packet_form & Packet_Form::Target_Status) == Packet_Form::Target_Status &&
                _target_status.target_num > 0 && (packet_form & Packet_Form::Target_Info == Packet_Form::Target_Info)) {
            return true;
        }
    }

    return false;
}

// get sensor system version
bool AP_RangeFinder_MMWRadar::get_sensor_version(uint32_t &sensor_version)
{
    uint8_t data_buf[MMWRADAR_DATA_BUFFER_SIZE+4] = {0};
    uint32_t start_times = AP_HAL::millis();

    data_buf[0] = MMWRADAR_START_SEQUENCE_L;
    data_buf[1] = MMWRADAR_START_SEQUENCE_H;
    data_buf[2] = MMWRADAR_MSGID_CONFIGURATION_L;
    data_buf[3] = MMWRADAR_MSGID_CONFIGURATION_H;
    data_buf[4] = MMWRADAR_DATATYPE_SENSOR_VERSION | (MMWRADAR_CON_READ << 7);

    for (int i=0; i<(MMWRADAR_DATA_BUFFER_SIZE+4); i++) {
        uart->write(data_buf[i]);
    }
    uart->flush();

    while (!uart->available()) {
        if (AP_HAL::millis() - start_times > 200) {
            return false;
        }
    }

    // reset all variable before touch uart
    reset_param();

    if (recv_packet()) {
        if (parse() & AP_RangeFinder_MMWRadar::Sensor_Back) {
            sensor_version = (_master_version << 16) | (_second_version << 8) | _step_version;
            return true;
        }
    }

    return false;
}


// handle - parse packet
AP_RangeFinder_MMWRadar::Packet_Form AP_RangeFinder_MMWRadar::parse()
{
    uint8_t packet_form = 0;
    for (int i=0; i<_valid_data_packet; i++)
    {
        uint16 msgid = _valid_data_buf[i][0] | (_valid_data_buf[i][1] << 8);

        switch (msgid) {
            case MMWRADAR_MSGID_SENSOR_BACK:
                uint8_t datatype = get_data_from_buf(i, MMWRADAR_READ_STATUS_OFFSET, ~MMWRADAR_READ_STATUS_BIT);
                uint8_t read_status = get_data_from_buf(i, MMWRADAR_READ_STATUS_OFFSET, MMWRADAR_READ_STATUS_BIT);
                switch (datatype) {
                    case MMWRADAR_DATATYPE_SENSOR_VERSION:
                        if (read_status == MMWRADAR_READ_STATUS_BIT) {
                            _master_version = get_data_from_buf(i, MMWRADAR_MASTER_VERSION_OFFSET);
                            _second_version = get_data_from_buf(i, MMWRADAR_SECOND_VERSION_OFFSET);
                            _step_version = get_data_from_buf(i, MMWRADAR_STEP_VERSION_OFFSET);

                            packet_form |= Packet_Form::Sensor_Back;
                        }
                        break;
                    case MMWRADAR_DATATYPE_SENSOR_ID:
                    case MMWRADAR_DATATYPE_SENSOR_SWITCH:
                    case MMWRADAR_DATATYPE_FILTER:
                    case MMWRADAR_DATATYPE_SAVE_PARAMETER:
                    case MMWRADAR_DATATYPE_INTEST:
                    default:
                        break;
                }
                break;

            case MMWRADAR_MSGID_SENSOR_STATUS:
                packet_form |= Packet_Form::Sensor_Status;
                break;
            case MMWRADAR_MSGID_TARGET_STATUS:
                _target_status.target_num = get_data_from_buf(i, MMWRADAR_TARGET_STATUS_TARGETNUM_OFFSET);
                _target_status.rollcount = get_data_from_buf(i, MMWRADAR_TARGET_STATUS_ROLLCOUNT_OFFSET, MMWRADAR_TARGET_STATUS_ROLLCOUNT_BIT);

                packet_form |= Packet_Form::Target_Status;
                break;
            case MMWRADAR_MSGID_TARGET_INFO:
                _target_info.index = (uint8_t)get_data_from_buf(i, MMWRADAR_TARGET_INFO_TARGETID_OFFSET);
                _target_info.rcs_cm = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RCS_OFFSET) * 50 - 5000;

                char range_l = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RANGE_L_OFFSET);
                char range_h = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RANGE_H_OFFSET);
                _target_info.range_cm = (range_h << 8) | range_l;

                _target_info.snr = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RCS_OFFSET) - 127;

                packet_form |= Packet_Form::Target_Info;
                break;
            default:
                packet_form |= Packet_Form::None;
                break;
        }
    }
}

// read valid packet from uart
bool AP_RangeFinder_MMWRadar::recv_packet()
{
    uint16_t nbytes = uart->available();
    uint8_t offset = 0, num = 0;
    uint16_t overflow = 0;

    while (nbytes-- > 0) {
        char c = uart->read();
        overflow++;
        if (nbytes > MMWRADAR_DATA_BUFFER_SIZE) {
            if (c == MMWRADAR_START_SEQUENCE_L) {
                nbytes--;
                overflow++;

                if (uart>read() == MMWRADAR_START_SEQUENCE_H) {
                    c = uart->read();
                    char next_c = uart->read();
                    while (!(c == MMWRADAR_END_SEQUENCE_L && next_c == MMWRADAR_END_SEQUENCE_H)) {
                        _valid_data_buf[num][offset++] = c;
                        c = next_c;
                        next_c = uart->read();
                        if (offset > 10) {
                            break;
                        }
                    }

                    nbytes -= (offset + 2);
                    overflow += (offset + 2);
                    if (offset == MMWRADAR_DATA_BUFFER_SIZE) {
                        num++;
                    }
                }
            }
        } else if (num > 0) {
            // exit circle if remaining bytes less than one packet size.
            break;
        } else {
            // return if packet is not complete.
            return false;
        }

        // advoid receive error
        if (overflow > 100) {
            return false;
        }
    }

    _valid_data_packet = num;

    return true;
}

void AP_RangeFinder_MMWRadar::reset_param()
{
    for (int i=0; i<MMWRADAR_DATA_BUFFER_NUM; i++) {
        for (int j=0; j<MMWRADAR_DATA_BUFFER_SIZE; j++) {
            _valid_data_buf[i][j] = 0;
        }
    }

    _valid_data_packet = 0;
    memset(&_target_status, 0, sizeof(AP_RangeFinder_MMWRadar::Target_Status));
    memset(&_target_info, 0, sizeof(AP_RangeFinder_MMWRadar::Target_Info));
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MMWRadar::update(void)
{
    if (get_reading()) {
        state.distance_cm = _target_info.range_cm;
        state.snr = _target_info.snr;
        state.rcs_cm = _target_info.rcs_cm;
        // update range_valid state based on distance measured
        _last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - _last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
