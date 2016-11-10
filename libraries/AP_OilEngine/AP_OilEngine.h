/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PID library (2-axis)
#include <AC_PID/AC_P.h>               // P library
#include <AP_Motors/AP_Motors.h>          // motors library
#include <AP_Vehicle/AP_Vehicle.h>         // common vehicle parameters
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#define AC_OIL_ENGINE_PWMIN0_PIN 3  // default is that AUX3,4 use to oil engine pwm input
#define AC_OIL_ENGINE_PWMIN1_PIN 4

#define AC_OIL_ENGINE_MAX_PWMOUT 1000

#define OIL_ENGINE_MOTOR_NUM 2

class AP_OilEngine
{
public:

    /// Constructor
    AP_OilEngine(const AP_Motors& motors, AC_PID& pid_motor_speed);

    ///
    /// initialisation functions
    ///
    void init();

    /// feedback oil engine motor rpm
    uint32_t feedback_motors_rpm(uint8_t motor_index);

    /// feedback oil engine motor pwm in Hz
    uint32_t feedback_motors_pwm_Hz(uint8_t motor_index);

    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    ///     updates z axis accel controller's D term filter
    void set_dt(float delta_sec);
    float get_dt() const { return _dt; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct motor_state {
        // convert period to oil engine motor rpm (r/min)
        uint32_t _rpm;
        uint32_t _pwm_in_Hz;
    };

    // general purpose flags
    struct poscontrol_flags {
            uint16_t recalc_leash_z     : 1;    // 1 if we should recalculate the z axis leash length
            uint16_t recalc_leash_xy    : 1;    // 1 if we should recalculate the xy axis leash length
            uint16_t reset_desired_vel_to_pos   : 1;    // 1 if we should reset the rate_to_accel_xy step
            uint16_t reset_rate_to_accel_xy     : 1;    // 1 if we should reset the rate_to_accel_xy step
            uint16_t reset_accel_to_lean_xy     : 1;    // 1 if we should reset the accel to lean angle step
            uint16_t reset_rate_to_accel_z      : 1;    // 1 if we should reset the rate_to_accel_z step
            uint16_t reset_accel_to_throttle    : 1;    // 1 if we should reset the accel_to_throttle step of the z-axis controller
            uint16_t freeze_ff_xy       : 1;    // 1 use to freeze feed forward during step updates
            uint16_t freeze_ff_z        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
            uint16_t use_desvel_ff_z    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step
            uint16_t use_auto_imitation : 1;
    } _flags;

    const AP_Motors&            _motors;

    // references to pid controllers
    AC_PID&     _pid_motor_speed;

    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    float       _speed_cms;             // max horizontal speed in cm/s

    // position controller internal variables
    Vector3f    _vel_desired;           // desired velocity in cm/s
    Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
    Vector3f    _vel_error;             // error between desired and actual acceleration in cm/s
    Vector3f    _vel_last;              // previous iterations velocity in cm/s
    Vector3f    _accel_target;          // desired acceleration in cm/s/s  // To-Do: are xy actually required?
    Vector3f    _accel_error;           // desired acceleration in cm/s/s  // To-Do: are xy actually required?
    Vector3f    _accel_feedforward;     // feedforward acceleration in cm/s/s
    LowPassFilterFloat _vel_error_filter;   // low-pass-filter on z-axis velocity error

    Vector2f    _accel_target_jerk_limited; // acceleration target jerk limited to 100deg/s/s
    LowPassFilterVector2f _accel_target_filter; // acceleration target filter

    AP_Int8         _trigger_type;      // 0:Servo,1:Relay
    AP_Int8         _trigger_duration;  // duration in 10ths of a second that the camera shutter is held open
    AP_Int8         _relay_on;          // relay value to trigger camera
    AP_Int16        _servo_on_pwm;      // PWM value to move servo to when shutter is activated
    AP_Int16        _servo_off_pwm;     // PWM value to move servo to when shutter is deactivated
    uint8_t         _trigger_counter;   // count of number of cycles shutter has been held open
    AP_Relay       *_apm_relay;         // pointer to relay object from the base class Relay.

    bool            setup_feedback_callback(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static void     capture0_callback(void *context, uint32_t chan_index,
                                      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
    static void     capture1_callback(void *context, uint32_t chan_index,
                                      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
#endif

    // pin number for accurate camera feedback messages
    AP_Int8         _pwm_input0_pin;
    AP_Int8         _pwm_input1_pin;
    AP_Int16        _max_pwm_out;

    static volatile struct motor_state _motors_state[OIL_ENGINE_MOTOR_NUM];

    bool            _valid_rpm;
};
