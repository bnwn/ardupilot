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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <drivers/drv_hrt.h>
#endif

#define OIL_ENGINE_PWMIN0_PIN 3  // default is that AUX3,4 use to oil engine pwm input
#define OIL_ENGINE_PWMIN1_PIN 4

#define OIL_ENGINE_MAX_ANGLE 1000
#define OIL_ENGINE_MAX_RPM 5000.0f
#define OIL_ENGINE_MOTOR_NUM 2

#define THROTTLE_TO_RPM_SCALE 200.0f

#define PWM_INPUT_TIMEOUT_US 500000.0f

#define PWM_INPUT_MAX_HZ 500.0f

class AP_OilEngine
{
public:

    /// Constructor
    AP_OilEngine(AC_PID& pid_motor1_rpm, AC_PID& pid_motor2_rpm);

    ///
    /// initialisation functions
    ///
    void init();

    /// control oil engine servo output
    void control();

    /// calc rpm vi period
    void calc_rpm();

    /// feedback oil engine motor rpm
    uint32_t feedback_motors_rpm(uint8_t motor_index);

    /// feedback oil engine motor pwm in Hz
    uint32_t feedback_motors_pwm_Hz(uint8_t motor_index);

    /// set radio pwm in
    void set_radio_in(int16_t radio_in, int16_t scale);

    /// desired throttle convert to rpm
    void throttle_to_rpm();

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct motor_state {
        // convert period to oil engine motor rpm (r/min)
        float _rpm;
        float _pwm_in_Hz;
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

    bool            setup_feedback_callback(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static void     capture0_callback(void *context, uint32_t chan_index,
                                      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
    static void     capture1_callback(void *context, uint32_t chan_index,
                                      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
#endif

    // references to pid controllers
    AC_PID&     _pid_motor1_rpm;
    AC_PID&     _pid_motor2_rpm;
    float       _throttle_desired;
    float       _rpm_desired;
    int16_t     _servo1_angle_desired;
    int16_t     _servo2_angle_desired;

    AP_Float       _max_rpm;             // max rpm in r/min
    AP_Int16       _max_frequency;
    // pin number for accurate camera feedback messages
    AP_Int8         _pwm_input0_pin;
    AP_Int8         _pwm_input1_pin;
    AP_Int16        _max_angle;
    AP_Float        _throttle_to_rpm_scale;

    struct motor_state _motors_state[OIL_ENGINE_MOTOR_NUM];
    volatile static uint32_t _period[OIL_ENGINE_MOTOR_NUM];

    bool            _valid_rpm;

    volatile static hrt_abstime _last_irq_time[OIL_ENGINE_MOTOR_NUM];
};
