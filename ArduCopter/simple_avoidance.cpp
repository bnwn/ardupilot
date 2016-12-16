// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#define AVOID_OBSTACLE_DISTANCE_CM 500
// Function that control a simple avoidance obstacle
// ----------------------------------------------------------------------------

#if AVOID_OBSTACLE == ENABLED
bool Copter::check_obstacle(void)
{
    if (mmwradar_state.range_cm < AVOID_OBSTACLE_DISTANCE_CM && mmwradar_state.range_healthy && mmwradar_state.enabled) {
        return true;
    }
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
            if (set_mode(BRAKE, MODE_REASON_AVOIDANCE)) {
                is_avoid_obstacle = true;
                //printf ("BRAKE!!\n");
            }
        }
    } else if (is_avoid_obstacle) {
        is_avoid_obstacle = false;
        if (control_mode == BRAKE) {
            set_mode(prev_control_mode, MODE_REASON_AVOIDANCE);
            //printf("exit avoid obstacle!!\n");
            //printf("and current mode is: %d\n", control_mode);
        }
    }
}

#endif

