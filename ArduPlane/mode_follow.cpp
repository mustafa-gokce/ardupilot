#include "mode.h"
#include "Plane.h"

#if MODE_FOLLOW_ENABLED == ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 */

// initialise follow mode
bool ModeFollow::_enter()
{
    // do not change the mode if follow is not enabled
    if (!plane.g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Enable FOLLOW mode");
        return false;
    }

    // start loitering before proceeding to the target
    plane.guided_throttle_passthru = false;
    Location loc{plane.current_loc};
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()), plane.quadplane.stopping_distance());
    }
#endif
    plane.set_guided_WP(loc);

    // notify user
    gcs().send_text(MAV_SEVERITY_DEBUG, "Started follow mode");
    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::_exit()
{
    // notify user
    gcs().send_text(MAV_SEVERITY_DEBUG, "Exited follow mode");
}

void ModeFollow::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeFollow::navigate()
{
    /*
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.prev_WP_loc;
    plane.next_WP_loc.offset_bearing(plane.gps.ground_course_cd()*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
    plane.prev_WP_loc.set_alt_cm(6000, Location::AltFrame::ABOVE_HOME);
    plane.next_WP_loc.set_alt_cm(6000, Location::AltFrame::ABOVE_HOME);
     */

    plane.prev_WP_loc.lat = -353612420;
    plane.prev_WP_loc.lng = 1491596870;
    plane.prev_WP_loc.set_alt_cm(6000, Location::AltFrame::ABOVE_HOME);

    plane.next_WP_loc.lat = -353608830;
    plane.next_WP_loc.lng = 1491661860;
    plane.next_WP_loc.set_alt_cm(6000, Location::AltFrame::ABOVE_HOME);

    plane.auto_state.crosstrack = true;
    plane.auto_state.next_wp_crosstrack = true;
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
