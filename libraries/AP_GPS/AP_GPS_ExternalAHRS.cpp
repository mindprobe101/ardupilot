/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  ExternalAHRS GPS driver
//
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include "AP_GPS_ExternalAHRS.h"

#if HAL_EXTERNAL_AHRS_ENABLED

static uint16_t external_dop_to_gps_dop(const float dop)
{
    if (!isfinite(dop) || is_negative(dop)) {
        return GPS_UNKNOWN_DOP;
    }
    return uint16_t(MIN(dop * 100.0f, float(GPS_UNKNOWN_DOP - 1)));
}

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_ExternalAHRS::read(void)
{
    if (new_data) {
        new_data = false;
        return true;
    }
    return false;
}

// handles an incoming ExternalAHRS message and sets
// corresponding gps data appropriately;
void AP_GPS_ExternalAHRS::handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt)
{
    check_new_itow(pkt.ms_tow, sizeof(pkt));

    state.time_week = pkt.gps_week;
    state.time_week_ms = pkt.ms_tow;
    if (pkt.fix_type == AP_GPS_FixType::NO_GPS) {
        state.status = AP_GPS::NO_FIX;
    } else {
        state.status = (AP_GPS::GPS_Status)pkt.fix_type;
    }
    state.num_sats = pkt.satellites_in_view;

    Location loc = {};
    loc.lat = pkt.latitude;
    loc.lng = pkt.longitude;
    loc.alt = pkt.msl_altitude;

    state.location = loc;
    state.hdop = external_dop_to_gps_dop(pkt.hdop);
    state.vdop = external_dop_to_gps_dop(pkt.vdop);

    state.velocity.x = pkt.ned_vel_north;
    state.velocity.y = pkt.ned_vel_east;
    state.velocity.z = pkt.ned_vel_down;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = pkt.have_speed_accuracy;
    state.have_horizontal_accuracy = pkt.have_horizontal_accuracy;
    state.have_vertical_accuracy = pkt.have_vertical_accuracy;
    state.have_vertical_velocity = pkt.have_vertical_velocity;
    state.have_undulation = pkt.have_undulation;

    state.horizontal_accuracy = pkt.horizontal_pos_accuracy;
    state.vertical_accuracy = pkt.vertical_pos_accuracy;
    state.speed_accuracy = pkt.speed_accuracy;
    state.undulation = pkt.undulation;

    // ExternalAHRS GPS messages do not provide a moving-baseline vector.
    state.rtk_baseline_coords_type = 0;
    state.rtk_baseline_x_mm = 0;
    state.rtk_baseline_y_mm = 0;
    state.rtk_baseline_z_mm = 0;
    state.rtk_accuracy = 0;
    state.rtk_iar_num_hypotheses = 0;

    if (state.status >= AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
        state.rtk_time_week_ms = pkt.ms_tow;
        state.rtk_week_number = pkt.gps_week;
        state.rtk_age_ms = pkt.rtk_age_ms;
        state.rtk_num_sats = pkt.rtk_num_sats;
    } else {
        state.rtk_time_week_ms = 0;
        state.rtk_week_number = 0;
        state.rtk_age_ms = 0;
        state.rtk_num_sats = 0;
    }

    state.last_gps_time_ms = AP_HAL::millis();

    new_data = true;
}

/*
  return velocity lag in seconds
 */
bool AP_GPS_ExternalAHRS::get_lag(float &lag_sec) const
{
    // fixed assumed lag
    lag_sec = 0.11;
    return true;
}

#endif // HAL_EXTERNAL_AHRS_ENABLED
