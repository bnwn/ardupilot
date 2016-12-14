// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#define AVOID_OBSTACLE_DISTANCE_CM 300
#define WANRING_TIME_MS 200
// Function that control a simple avoidance obstacle
// ----------------------------------------------------------------------------

#if AVOID_OBSTACLE == ENABLED
bool Copter::check_obstacle(void)
{
    uint32_t now = AP_HAL::millis();
    static uint32_t distance_waring_time_ms = now;
    if (mmwradar_state.range_cm < AVOID_OBSTACLE_DISTANCE_CM && mmwradar_state.range_healthy && mmwradar_state.enabled) {
        if (now - distance_waring_time_ms > WANRING_TIME_MS) {
            return true;
        } else {
            return false;
        }
    }
    distance_waring_time_ms = now;
    return false;
}

void Copter::avoidance_obstacle(void)
{
    static bool is_avoid_obstacle = false;
    if (check_obstacle()) {
        if (!is_avoid_obstacle &&
                //position_ok() &&
                (control_mode == AUTO ||
                 control_mode == POINT_ATOB ||
                 control_mode == LOITER)) {
            if (set_mode(POSHOLD, MODE_REASON_AVOIDANCE)) {
                is_avoid_obstacle = true;
                printf ("poshold!!\n");
            }
        }
    } else if (is_avoid_obstacle) {
        if (set_mode(prev_control_mode, MODE_REASON_AVOIDANCE)) {
            is_avoid_obstacle = false;

            printf("exit avoid obstacle!!\n");
            printf("and current mode is: %d\n", control_mode);
        }
    }
}

#endif

