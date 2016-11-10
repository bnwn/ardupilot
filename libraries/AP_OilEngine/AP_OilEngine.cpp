// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_OilEngine.h"
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_pwm_output.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_OilEngine::var_info[] = {

    // @Param: MAX_PWMOUT
    // @User: Enigma
    AP_GROUPINFO("MAX_PWMOUT",  0, AP_OilEngine, _max_pwm_out, AC_OIL_ENGINE_MAX_PWMOUT),

    // @Param: PWMIN0
    // @User: Enigma
    AP_GROUPINFO("PWMIN0",  1, AP_OilEngine, _pwm_input0_pin, AC_OIL_ENGINE_PWMIN0_PIN),

    // @Param: PWMIN1
    // @User: Enigma
    AP_GROUPINFO("PWMIN1",  2, AP_OilEngine, _pwm_input1_pin, AC_OIL_ENGINE_PWMIN1_PIN),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/*
  static var for PX4 callback
 */
volatile struct AP_OilEngine::Motor_State AP_OilEngine::_motors_state[OIL_ENGINE_MOTOR_NUM];

void AP_OilEngine::AP_OilEngine(const AP_Motors &motors, AC_PID &pid_motor_speed) :
    _motors_state{0},
    _valid_rpm(flase)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OilEngine::init()
{
    if (!setup_feedback_callback())
    {
        printf("capture setup error");
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Oil Engine Setup Failure");
    } else {
        _valid_rpm = true;
    }
}

uint32_t AP_OilEngine::feedback_motors_rpm(uint8_t motor_index)
{
    return _motors_state[motor_index]._rpm;
}

uint32_t AP_OilEngine::feedback_motors_pwm_Hz(uint8_t motor_index)
{
    return _motors_state[motor_index]._pwm_in_Hz;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
  callback for timer capture on PX4
 */
void AP_OilEngine::capture0_callback(void *context, uint32_t chan_index,
                                     hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
    hrt_abstime now = hrt_absolute_time();

    uint32_t period = hrt_absolute_time - edge_time;

    _motors_state[0]._pwm_in_Hz = 1000000UL / period;
    _motors_state[0]._rpm = 3600 * _motors_state[0]._pwm_in_Hz;
}

void AP_OilEngine::capture1_callback(void *context, uint32_t chan_index,
                                     hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
    hrt_abstime now = hrt_absolute_time();

    uint32_t period = hrt_absolute_time - edge_time;

    _motors_state[1]._pwm_in_Hz = 1000000UL / period;
    _motors_state[1]._rpm = 3600 * _motors_state[0]._pwm_in_Hz;
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
        return;
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

        if (up_input_capture_set(_pwm_input0_pin-1, Falling, 0, capture_callback0, this) != 0) {
            close(fd);
            return false;
        }
        if (up_input_capture_set(_pwm_input1_pin-1, Falling, 0, capture_callback1, this) != 0) {
            close(fd);
            return false;
        }
        close(fd);
    }
}
