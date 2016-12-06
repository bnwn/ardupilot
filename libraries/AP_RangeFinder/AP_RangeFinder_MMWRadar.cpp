// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_RangeFinder_MMWRadar.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

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
    AP_RangeFinder_MMWRadar *sensor
        = new AP_RangeFinder_MMWRadar(_ranger, instance, _state, serial_manager);
    if (!sensor || !sensor->start_reading()) {
        delete sensor;
        return nullptr;
    }
    // give time for the sensor to process the request
    hal.scheduler->delay(50);
    uint16_t reading_cm;

    if (!sensor->get_reading(reading_cm)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// read - return last value measured by sensor
bool AP_RangeFinder_MMWRadar::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (float)atof(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c) || c == '.') {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    // we need to write a byte to prompt another reading
    uart->write('d');

    if (count == 0) {
        return false;
    }
    reading_cm = 100 * sum / count;
    return true;
}

/* get sensor system version */
bool AP_RangeFinder_MMWRadar::get_sensor_version(uint32_t &sensor_version)
{
    uint8_t data_buf[MMWRADAR_DATA_BUFFER_SIZE + 4] = {0};
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
        if (AP_HAL::millis() - start_times > 20) {
            return false;
        }
    }

    uint16_t nbytes = uart->available();

    while (nbytes-- > 0)
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MMWRadar::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
