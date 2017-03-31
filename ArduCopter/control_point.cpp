/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * Init and run calls for point A ot B fligt mode
 */

bool Copter::point_init(bool ignore_checks)
{
    if ((position_ok() && mission.is_valid_point()) || ignore_checks) {
        auto_mode = Auto_Loiter;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce change of flips)
        if (motors.armed() && ap.land_complete) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Point: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

        // clear guided limits
        guided_limit_clear();

        pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

        // start/resume the point mission
        mission.point_atob_start();
        return true;

    }

    return false;
}

void Copter::point_run()
{
    // call the correct auto controller
    switch (auto_mode) {

    case Auto_TakeOff:
        auto_takeoff_run();
        break;

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Land:
        auto_land_run();
        break;

    case Auto_RTL:
        auto_rtl_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_Spline:
        auto_spline_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED == ENABLED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;
    }
}


