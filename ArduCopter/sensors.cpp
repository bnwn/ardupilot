// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

void Copter::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors.set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = (rangefinder.num_sensors() >= 1);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.alt_healthy = ((rangefinder.status() == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count() >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm();

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    rangefinder_state.alt_cm_filter = rangefinder_state.alt_cm_filt.get();

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

void Copter::init_flowmeter(void)
{
#if FLOWMETER_ENABLED == ENABLED
    flowmeter_state.enabled = flowmeter.init();
    flowmeter_state.flowrate_filt.set_cutoff_frequency(FLOWMETER_FILT_HZ);
#endif
//    test
//    while(1) {
//        usleep(100000);
//        if (get_pesticide_remaining()) {
//            printf("flowrate_rel: %d \n  pluse rate: %d \n", flowmeter_state.flowrate, flowmeter.pluse_rate());
//        } else {
//            printf("pesticide empty! \n");
//        }
//    }
}

// return flowrate from flowmeter measurement
void Copter::read_flowmeter(void)
{
#if FLOWMETER_ENABLED == ENABLED
    flowmeter.update();

    flowmeter_state.healthy = (flowmeter.has_data() && (flowmeter.range_valid_count() >= FLOWMETER_HEALTH_MAX));
    flowmeter_state.pesticide_check_valid = flowmeter.get_state();
    flowmeter_state.flowrate = flowmeter.flowrate();

    // filter flowmeter
    uint32_t now = AP_HAL::millis();

    if (flowmeter_state.healthy) {
        if (now - flowmeter_state.last_healthy_ms > PESTICIDE_EMPTY_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            flowmeter_state.flowrate_filt.reset(flowmeter_state.flowrate);
        } else {
            flowmeter_state.flowrate_filt.apply(flowmeter_state.flowrate, 0.0f);
        }
        flowmeter_state.last_healthy_ms = now;
    }
#else
    flowmeter_state.enable = false;
    flowmeter_state.healthy = false;
    flowmeter_state.flowrate = 0.0f;
#endif
}

bool Copter::flowmeter_ok(void)
{
    return (flowmeter_state.enabled && flowmeter_state.healthy);
}

bool Copter::get_pesticide_remaining(void)
{
    read_flowmeter();

    if (flowmeter_ok()) {
        uint32_t now = AP_HAL::millis();

        if (flowmeter_state.pesticide_check_valid && flowmeter_state.flowrate <= PESTICIDE_RTL_REMAINING) {
            if (flowmeter_state.pesticide_empty_time == 0) {
                flowmeter_state.pesticide_empty_time = now;

            } else if ((now - flowmeter_state.pesticide_empty_time) > PESTICIDE_EMPTY_TIMEOUT_MS) {
                flowmeter_state.pesticide_empty_time = 0;
                flowmeter_state.pesticide_check_valid = false;
                flowmeter_state.flowrate = 0.0f;
                flowmeter_state.flowrate_filt.reset(flowmeter_state.flowrate);
                flowmeter.disable();
                return false;
            }
        } else {
            // reset pesticide empty time
            flowmeter_state.pesticide_empty_time = 0;
        }
    }

    return true;
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

// initialise compass
void Copter::init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println("COMPASS INIT ERROR");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
void Copter::read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }

    // update motors with voltage and current
    if (battery.get_type() != AP_BattMonitor::BattMonitor_TYPE_NONE) {
        motors.set_voltage(battery.voltage());
    }
    if (battery.has_current()) {
        motors.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Copter::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

void Copter::compass_cal_update()
{
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
#ifdef CAL_ALWAYS_REBOOT
    if (compass.compass_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

#if EPM_ENABLED == ENABLED
// epm update - moves epm pwm output back to neutral after grab or release is completed
void Copter::epm_update()
{
    epm.update();
}
#endif

/*
  update AP_Button
 */
void Copter::button_update(void)
{
    g2.button.update();
}
