// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_RangeFinder_MMWRadar.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MMWRadar::AP_RangeFinder_MMWRadar(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _data_buf_offset(0),
    _valid_data_packet(0),
    _master_version(0),
    _second_version(0),
    _step_version(0),
    _last_reading_ms(0),
    _packet_form(0)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MMWRadar, 0);
    if (uart != nullptr) {
        uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_MMWRadar, 0);
        printf ("begin mmwradar uart. baudrate is: %d \n", baudrate);
        uart->begin(baudrate);
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
            = new AP_RangeFinder_MMWRadar(ranger, instance, _state, serial_manager);
        uint32_t version;

        printf("find serial for mmwradar.\n");
        if (sensor && sensor->get_sensor_version(version)) {
        //if (sensor && sensor->get_reading()) {
            printf("mmwradar sensor init success. instance: %d\n", instance);
            return sensor;
        }
        delete sensor;
    }

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
        _packet_form = parse();

        if ((_packet_form & Packet_Form::Sensor_Status) == Packet_Form::Sensor_Status ||
                (_packet_form & Packet_Form::Target_Status) == Packet_Form::Target_Status ||
                (_packet_form & Packet_Form::Target_Info) == Packet_Form::Target_Info) {
            return true;
        }
    }

    return false;
}

// get sensor system version
bool AP_RangeFinder_MMWRadar::get_sensor_version(uint32_t &sensor_version)
{
    char data_buf[MMWRADAR_DATA_BUFFER_SIZE+4] = {0};
    uint32_t start_times = AP_HAL::millis();

    data_buf[0] = MMWRADAR_START_SEQUENCE_L;
    data_buf[1] = MMWRADAR_START_SEQUENCE_H;

    hal.scheduler->delay(20);
    while (!uart->available()) {
        if (AP_HAL::millis() - start_times > 200) {
            printf("wait sensor back timeout.\n");
            return false;
        }
    }

    start_times = AP_HAL::millis();
    char c = uart->read();
    bool is_mmwradar_packet = false;
    while (!is_mmwradar_packet) {
        if (AP_HAL::millis() - start_times > 500) {
            printf("wait sensor packet timeout.\n");
            return false;
        }
        if (c == MMWRADAR_START_SEQUENCE_L) {
            c = uart->read();
            if (c == MMWRADAR_START_SEQUENCE_H) {

                for (int i=0; i<(MMWRADAR_DATA_BUFFER_SIZE+2); i++) {
                    data_buf[i+2] = uart->read();
                }
                if (data_buf[MMWRADAR_DATA_BUFFER_SIZE+2] == MMWRADAR_END_SEQUENCE_L && data_buf[MMWRADAR_DATA_BUFFER_SIZE+3] == MMWRADAR_END_SEQUENCE_H) {
                    is_mmwradar_packet = true;
                }
            }
        } else {
            c = uart->read();
        }
    }

    return true;
}

// read valid packet from uart
bool AP_RangeFinder_MMWRadar::recv_packet()
{
    uint16_t nbytes = uart->available();
    uint8_t num = 0;

    int i=0;
    for (i=0; i<nbytes; i++) {
        if ((i+_data_buf_offset) < MMWRADAR_UART_BUFFER_SIZE) {
            _data_buf[i+_data_buf_offset] = uart->read();
        } else {
            break;
        }
    }
    _data_buf_offset += i;
    nbytes = _data_buf_offset;
    uint16_t current_offset = 0;
    while (current_offset < nbytes) {
        if ((nbytes - current_offset) >= (MMWRADAR_DATA_BUFFER_SIZE + 4)) {
            char c = _data_buf[current_offset++];
            if (c == MMWRADAR_START_SEQUENCE_L) {
                if (_data_buf[current_offset] == MMWRADAR_START_SEQUENCE_H) {
                    if (_data_buf[current_offset+MMWRADAR_END_REL_OFFSET] == MMWRADAR_END_SEQUENCE_L && _data_buf[current_offset+MMWRADAR_END_REL_OFFSET+1] == MMWRADAR_END_SEQUENCE_H) {
                        current_offset += MMWRADAR_DATA_REL_OFFSET;
                        for (int j=0; j<MMWRADAR_DATA_BUFFER_SIZE; j++) {
                            _valid_data_buf[num][j] = _data_buf[current_offset++];
                        }
                        num++;
                    }
                }
            }
        } else if (num > 0) {
            // exit circle if remaining bytes less than one packet size.
            break;
        } else {
            // return if packet is not complete.
            // printf("packet recv not complete.\n");
            return false;
        }
    }

    _data_buf_offset -= current_offset;
    for (int j=0; j<_data_buf_offset; j++) {
        _data_buf[j] = _data_buf[j+current_offset];
    }
    memset((_data_buf+_data_buf_offset), 0, (MMWRADAR_UART_BUFFER_SIZE - _data_buf_offset) * sizeof(char));
    _valid_data_packet = num;

    return true;
}

// handle - parse packet
uint16_t AP_RangeFinder_MMWRadar::parse()
{
    uint8_t packet_form = 0, instance = 0;
    char range_h, range_l, datatype, read_status, vel_h, vel_l;
    uint16_t sum_range = 0, sum_snr = 0, sum_rcs = 0, sum_vel = 0;
    static int last_rollcount = -1;
    for (int i=0; i<_valid_data_packet; i++)
    {
        uint16_t msgid = _valid_data_buf[i][0] | (_valid_data_buf[i][1] << 8);

        switch (msgid) {
            case MMWRADAR_MSGID_SENSOR_BACK:
                datatype = get_data_from_buf(i, MMWRADAR_READ_STATUS_OFFSET, ~MMWRADAR_READ_STATUS_BIT);
                read_status = get_data_from_buf(i, MMWRADAR_READ_STATUS_OFFSET, MMWRADAR_READ_STATUS_BIT);
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
                if (last_rollcount != -1) {
                    if (!((_target_status.rollcount == 0 && last_rollcount == 3) ||
                           (_target_status.rollcount - last_rollcount == 1))) {
                        packet_form |= Packet_Form::Status_error;
                    }
                }
                last_rollcount = _target_status.rollcount;
                packet_form |= Packet_Form::Target_Status;
                break;
            case MMWRADAR_MSGID_TARGET_INFO:
                _target_info.index = (uint8_t)get_data_from_buf(i, MMWRADAR_TARGET_INFO_TARGETID_OFFSET);
                sum_rcs += get_data_from_buf(i, MMWRADAR_TARGET_INFO_RCS_OFFSET) * 50 - 5000;

                range_l = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RANGE_L_OFFSET);
                range_h = get_data_from_buf(i, MMWRADAR_TARGET_INFO_RANGE_H_OFFSET);
                sum_range += (range_h << 8) | range_l;

                vel_l = get_data_from_buf(i, MMWRADAR_TARGET_INFO_VEL_L_OFFSET);
                vel_h = get_data_from_buf(i, MMWRADAR_TARGET_INFO_VEL_H_OFFSET, MMWRADAR_TARGET_INFO_VEL_H_BIT);
                sum_vel += ((vel_h << 8) | vel_l) * 5 - 3500;

                sum_snr += get_data_from_buf(i, MMWRADAR_TARGET_INFO_SNR_OFFSET) - 127;
                instance++;
                packet_form |= Packet_Form::Target_Info;
                break;
            default:
                packet_form |= Packet_Form::None;
                break;
        }
    }
    _target_info.rcs_cm = sum_rcs / instance;
    _target_info.range_cm = sum_range / instance;
    _target_info.snr = sum_snr / instance;
    _target_info.vel_cm = sum_vel / instance;
    return packet_form;
}

void AP_RangeFinder_MMWRadar::reset_param()
{
    for (int i=0; i<MMWRADAR_DATA_BUFFER_NUM; i++) {
        for (int j=0; j<MMWRADAR_DATA_BUFFER_SIZE; j++) {
            _valid_data_buf[i][j] = 0;
        }
    }

    _valid_data_packet = 0;
    _packet_form = 0;
    memset(&_target_status, 0, sizeof(AP_RangeFinder_MMWRadar::Target_Status_Type));
    memset(&_target_info, 0, sizeof(AP_RangeFinder_MMWRadar::Target_Info_Type));
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MMWRadar::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (get_reading()) {
        if (_packet_form & Packet_Form::Target_Info) {
            state.distance_cm = _target_info.range_cm;
            state.snr = _target_info.snr;
            state.rcs_cm = _target_info.rcs_cm;
            state.vel_cm = _target_info.vel_cm;
            update_status();

            // update range_valid state based on distance measured
            _last_reading_ms = AP_HAL::millis();
        }
        if (_packet_form & Packet_Form::Status_error) {
            printf("missed data.\n");
        }
    } else if (now - _last_reading_ms > MMWRADAR_READ_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
