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

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
 //  rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt_slide.set_sliding_param(10);
   rangefinder_state.enabled = (rangefinder.num_sensors() >= 1);

#if MMWRADAR_ENABLED == ENABLED
   mmwradar_state.enabled = rangefinder_state.enabled;
//   mmwradar_state.range_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
//   mmwradar_state.range_cm_filt_2p.set_cutoff_frequency(50, RANGEFINDER_WPNAV_FILT_HZ);
//   mmwradar_state.range_cm_filt_kalman.set_kalman_param(RANGEFINDER_KALMAN_P, RANGEFINDER_KALMNA_R);
//   mmwradar_state.range_cm_filt_slide.set_sliding_param(10);
#endif
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.alt_healthy = ((rangefinder.status(0) == RangeFinder::RangeFinder_Good)
                                     && (rangefinder.range_valid_count(0) >= RANGEFINDER_HEALTH_MAX)
                                     && (rangefinder.status(1) == RangeFinder::RangeFinder_Good)
                                     && (rangefinder.range_valid_count(1) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt;
    float vehicle_tilt = ahrs.pitch;
    if (rangefinder.service_tilt(0) == 0 && rangefinder.service_tilt(1) == 0) { // isn't mmwradar
        // correct alt for angle of the rangefinder
        temp_alt = (float)rangefinder.distance_cm() * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
        rangefinder_state.tilt_angle = ahrs.pitch;
    } else if (gps.ground_speed() < 1.0f && fabs(vehicle_tilt) < 0.0523f) {
        int16_t advance_alt = rangefinder.distance_cm(0);
        int16_t back_alt = rangefinder.distance_cm(1);
        float advance_tilt = rangefinder.service_tilt(0) * M_PI / 180 + vehicle_tilt;
        float back_tilt = rangefinder.service_tilt(1) * M_PI / 180 - vehicle_tilt;

        rangefinder_state.tilt_angle = MAX(fabs(advance_tilt), fabs(advance_tilt));
        temp_alt = (advance_alt * MAX(0.707f, cos(advance_tilt) * cos(ahrs.roll)) + back_alt * MAX(0.707f, cos(back_tilt) * cos(ahrs.roll))) / rangefinder.fuse_correct();
    } else {
        uint8_t rngfnd_num = 0;
        if (ahrs.pitch_sensor > 0 && ahrs.pitch_sensor < 18000) {
            rngfnd_num = 1;
            vehicle_tilt *= -1;
        }

        temp_alt = rangefinder.distance_cm(rngfnd_num);

        rangefinder_state.tilt_angle = rangefinder.service_tilt(rngfnd_num) * M_PI / 180 + vehicle_tilt;
        if (rangefinder_state.tilt_angle < 0) {
            rangefinder_state.tilt_angle += M_2PI;
        } else if (rangefinder_state.tilt_angle > M_2PI) {
            rangefinder_state.tilt_angle -= M_2PI;
        }
        temp_alt = (float)temp_alt * MAX(0.707f, cos(rangefinder_state.tilt_angle) * cos(ahrs.roll));
    }

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
            rangefinder_state.alt_cm_filt_slide.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.02f);
            rangefinder_state.alt_cm_filt_slide.apply(rangefinder_state.alt_cm);
            rangefinder_state.alt_cm_filt_median.apply(rangefinder_state.alt_cm);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    rangefinder_state.alt_cm_filter = rangefinder_state.alt_cm_filt.get();
    rangefinder_state.alt_cm_filter_median = rangefinder_state.alt_cm_filt_median.get();
    rangefinder_state.alt_cm_filter_slide = rangefinder_state.alt_cm_filt_slide.get();

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());


#if MMWRADAR_ENABLED == ENABLED
    mmwradar_state.range_healthy = ((rangefinder.mmwradar_status() == RangeFinder::RangeFinder_Good) && (rangefinder.mmwradar_valid_count()) >= RANGEFINDER_HEALTH_MAX);

    // get mmwradar value
    rangefinder.mmwradar_distance(mmwradar_state.range_cm, mmwradar_state.rcs_cm, mmwradar_state.snr, mmwradar_state.vel_cm);
#endif

    switch (g.rangefinder_filter.get()) {
        case 1:
            //mmwradar_state.range_cm = mmwradar_state.range_cm_filter;
            rangefinder_state.alt_cm = rangefinder_state.alt_cm_filter;
            break;
        case 2:
            rangefinder_state.alt_cm = rangefinder_state.alt_cm_filter_slide;
            break;
        case 3:
            rangefinder_state.alt_cm = rangefinder_state.alt_cm_filter_median;
            break;
        case 0:
        default:
            break;
    }


#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    //return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
    return rangefinder_alt_ok_auto();
}

bool Copter::rangefinder_alt_ok_auto()
{
    return rangefinder_state.alt_healthy;
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
//            printf("flowrate_rel: %d \n  pluse rate: %.6f \n", flowmeter_state.flowrate, flowmeter.pluse_rate());
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

//    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "rate:%.2f", flowmeter_state.flowrate * 0.01f);
    if (flowmeter_ok()) {
        uint32_t now = AP_HAL::millis();

        if (flowmeter_state.pesticide_check_valid && (flowmeter_state.flowrate <= flowmeter.rtl_flowrate())) {
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
        cliSerial->printf("COMPASS INIT ERROR\n");
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
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
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
        motors->set_voltage(battery.voltage());
        AP_Notify::flags.battery_voltage = battery.voltage();
    }
    if (battery.has_current()) {
        motors->set_current(battery.current_amps());
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
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
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

#if GRIPPER_ENABLED == ENABLED
// gripper update
void Copter::gripper_update()
{
    g2.gripper.update();
}
#endif

/*
  update AP_Button
 */
void Copter::button_update(void)
{
    g2.button.update();
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update proximity sensor
void Copter::update_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.update();
#endif
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.get_status() > AP_Proximity::Proximity_NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif
    if (copter.battery.healthy()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }


    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY);

    switch (control_mode) {
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case POSHOLD:
    case BRAKE:
    case THROW:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case ALT_HOLD:
    case GUIDED_NOGPS:
    case SPORT:
    case AUTOTUNE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (g.fs_batt_voltage > 0 || g.fs_batt_mah > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }



    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() == AP_GPS::NO_GPS) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ap.rc_receiver_present || failsafe.radio) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (!optflow.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (!precland.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (copter.failsafe.battery) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;                                                                    }
    
#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init beacons used for non-gps position estimates
void Copter::init_beacon()
{
    g2.beacon.init();
}

// update beacons
void Copter::update_beacon()
{
    g2.beacon.update();
}
