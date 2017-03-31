// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_OilEngine.h"
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <drivers/drv_input_capture.h>
#include <drivers/drv_pwm_output.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#endif

#define TEST_OIL

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_OilEngine::var_info[] = {

    // @Param: MAX_ANG
    // @User: Enigma
    AP_GROUPINFO("MAX_ANG",  0, AP_OilEngine, _max_angle, OIL_ENGINE_MAX_ANGLE),

    // @Param: PWMIN0
    // @User: Enigma
    AP_GROUPINFO("PWMIN0",  1, AP_OilEngine, _pwm_input0_pin, OIL_ENGINE_PWMIN0_PIN),

    // @Param: PWMIN1
    // @User: Enigma
    AP_GROUPINFO("PWMIN1",  2, AP_OilEngine, _pwm_input1_pin, OIL_ENGINE_PWMIN1_PIN),

    // @Param: THR2RPM
    // @User: Enigma
    AP_GROUPINFO("THR2RPM", 3, AP_OilEngine, _throttle_to_rpm_scale, THROTTLE_TO_RPM_SCALE),

    // @Param: MAX_RPM
    // @User: Enigma
    AP_GROUPINFO("MAX_RPM", 4, AP_OilEngine, _max_rpm, OIL_ENGINE_MAX_RPM),

    // @Param: MAX_FRE
    // @User: Enigma
    AP_GROUPINFO("MAX_FRE", 5, AP_OilEngine, _max_frequency, PWM_INPUT_MAX_HZ),

    AP_GROUPEND
};

/*
  static var for PX4 callback
 */
volatile uint32_t AP_OilEngine::_period[OIL_ENGINE_MOTOR_NUM];
volatile hrt_abstime AP_OilEngine::_last_irq_time[OIL_ENGINE_MOTOR_NUM];

AP_OilEngine::AP_OilEngine(AC_PID& pid_motor1_rpm, AC_PID& pid_motor2_rpm) :
    _pid_motor1_rpm(pid_motor1_rpm),
    _pid_motor2_rpm(pid_motor2_rpm),
    _throttle_desired(0.0f),
    _rpm_desired(0.0f),
    _servo1_angle_desired(0),
    _servo2_angle_desired(0),
    _valid_rpm(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    _motors_state[0]._pwm_in_Hz = _motors_state[0]._rpm = _motors_state[1]._pwm_in_Hz = _motors_state[1]._rpm = 0.0f;
    _period[0] = _period[1] = 0;
    _last_irq_time[0] = _last_irq_time[1] = hrt_absolute_time();
}

void AP_OilEngine::init()
{
    /* setup pwm input pin */
    if (!setup_feedback_callback())
    {
        printf("capture setup error");
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Oil Engine Setup Failure");
    } else {
        _valid_rpm = true;
    }

    /* setup pwm output servo */
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_oil_engine1, CH_7);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_oil_engine2, CH_8);

    // ensure servo are off
    RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_oil_engine1);
    RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_oil_engine2);
}

void AP_OilEngine::control()
{
    // exit immediately if the function has not been set-up for any servo or pwm input pin setup faliure
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_oil_engine1) || \
            !RC_Channel_aux::function_assigned(RC_Channel_aux::k_oil_engine2) || \
            !_valid_rpm) {
        return;
    }

    calc_rpm();

    float motor1_rpm_error = _rpm_desired - _motors_state[0]._rpm;
    float motor2_rpm_error = _rpm_desired - _motors_state[1]._rpm;

    _pid_motor1_rpm.set_input_filter_all(motor1_rpm_error);
    _pid_motor2_rpm.set_input_filter_all(motor2_rpm_error);

    _servo1_angle_desired = _pid_motor1_rpm.get_p() + _pid_motor1_rpm.get_d();
    _servo2_angle_desired = _pid_motor2_rpm.get_p() + _pid_motor1_rpm.get_d();

#ifdef TEST_OIL
    static int i = 0;
    i++;
    if (i > 50) {
        printf("\nCH9 in: %.6f, desired rpm: %.6f \n", _throttle_desired, _rpm_desired);
        printf("motor1: pwm Hz: %.6f, rpm: %.6f r/min \n", _motors_state[0]._pwm_in_Hz, _motors_state[0]._rpm);
        printf("motor2: pwm Hz: %.6f, rpm: %.6f r/min \n", _motors_state[1]._pwm_in_Hz, _motors_state[1]._rpm);
        i = 0;
    }
#endif

    // update servo position
    RC_Channel_aux::move_servo(RC_Channel_aux::k_oil_engine1, _servo1_angle_desired, 0, OIL_ENGINE_MAX_ANGLE);
    RC_Channel_aux::move_servo(RC_Channel_aux::k_oil_engine2, _servo2_angle_desired, 0, OIL_ENGINE_MAX_ANGLE);
}

void AP_OilEngine::calc_rpm()
{
    for (int i=0; i<OIL_ENGINE_MOTOR_NUM; i++) {
        if (_period[i] > 0 && (hrt_absolute_time() - _last_irq_time[i] < PWM_INPUT_TIMEOUT_US)) {
            _motors_state[i]._pwm_in_Hz = 1000000.0f / (float)_period[i];

            if (_motors_state[i]._pwm_in_Hz > _max_frequency) {
                _motors_state[i]._rpm = 0.0f;
            } else {
                _motors_state[i]._rpm = 60.0f * _motors_state[i]._pwm_in_Hz;
            }
        } else {
            _motors_state[i]._rpm = 0.0f;
            _motors_state[i]._pwm_in_Hz = 0.0f;
        }
    }
    throttle_to_rpm();
}

uint32_t AP_OilEngine::feedback_motors_rpm(uint8_t motor_index)
{
    return _motors_state[motor_index]._rpm;
}

uint32_t AP_OilEngine::feedback_motors_pwm_Hz(uint8_t motor_index)
{
    return _motors_state[motor_index]._pwm_in_Hz;
}

void AP_OilEngine::set_radio_in(int16_t radio_in, int16_t scale)
{
    // convert to range 0~1
    _throttle_desired = (float)radio_in / (float)scale;
}

void AP_OilEngine::throttle_to_rpm()
{
    _rpm_desired = _throttle_desired * _throttle_to_rpm_scale;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
  callback for timer capture on PX4
 */
void AP_OilEngine::capture0_callback(void *context, uint32_t chan_index,
                                     hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
    hrt_abstime now = hrt_absolute_time();
    _period[0] = now - _last_irq_time[0];
    _last_irq_time[0] = now;
}

void AP_OilEngine::capture1_callback(void *context, uint32_t chan_index,
                                     hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
    hrt_abstime now = hrt_absolute_time();
    _period[1] = now - _last_irq_time[1];
    _last_irq_time[1] = now;
}

#endif

/*
  setup a callback for a feedback pin. When on PX4 with the right FMU
  mode we can use the microsecond timer.
 */
bool AP_OilEngine::setup_feedback_callback(void)
{
    if (_pwm_input0_pin < 0 && _pwm_input1_pin < 0) {
        // invalid pin
        return false;
    }

    /*
      special case on PX4. We can use the fast timer support
     */
    int fd = open("/dev/px4fmu", 0);
    if (fd != -1) {
        if (ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_MODE_2PWM2CAP) != 0) {
            close(fd);
            return false;
        }

        if (up_input_capture_set(_pwm_input0_pin-1, Rising, 0, capture0_callback, this) != 0) {
            close(fd);
            return false;
        }
        if (up_input_capture_set(_pwm_input1_pin-1, Rising, 0, capture1_callback, this) != 0) {
            close(fd);
            return false;
        }
        close(fd);
    }

    return true;
}
