/*
   Copyright (C) 2025 Kraus Hamdani Aerospace Inc. All rights reserved.

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
/*
  support for serial connected SBG INS
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_SBG.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Common/time.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

constexpr uint8_t AP_ExternalAHRS_SBG::SBG_PACKET_SYNC1;
constexpr uint8_t AP_ExternalAHRS_SBG::SBG_PACKET_SYNC2;
constexpr uint8_t AP_ExternalAHRS_SBG::SBG_PACKET_ETX;
constexpr uint16_t AP_ExternalAHRS_SBG::SBG_PACKET_PAYLOAD_SIZE_MAX;
constexpr uint16_t AP_ExternalAHRS_SBG::SBG_PACKET_OVERHEAD;

AP_ExternalAHRS_SBG::sbgMessage::sbgMessage(const uint8_t msgClass_, const uint8_t msgId_) :
    msgid(msgId_),
    msgclass(msgClass_)
{
}

AP_ExternalAHRS_SBG::sbgMessage::sbgMessage(const uint8_t msgClass_, const uint8_t msgId_,
                                            const uint8_t *payload, const uint16_t payload_len) :
    msgid(msgId_),
    msgclass(msgClass_)
{
    if (payload != nullptr && payload_len <= sizeof(data)) {
        memcpy(data, payload, payload_len);
        len = payload_len;
    }
}

// constructor
AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    initialise_gnss_pos(cached.sbg.gnssPos);
    initialise_utc(cached.sbg.utc);

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SBG ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);


    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
    
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "SBG", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("SBG Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG ExternalAHRS initialised");
}

void AP_ExternalAHRS_SBG::update_thread()
{
    hal.scheduler->delay(1000);
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);

    if (uart == nullptr) {
        return;
    }

    uart->begin(baudrate, 1024, 1024);

    setup_complete = true;

    while (true) {
        const bool received_data = check_uart();
        hal.scheduler->delay_microseconds(received_data ? 100 : 250);
        const uint32_t now_ms = AP_HAL::millis();
        if (option_is_set(AP_ExternalAHRS::OPTIONS::SBG_EKF_AS_GNSS)) {
            publish_ekf_gps(now_ms);
        }

        if (cached.sbg.deviceInfo.firmwareRev == 0 && now_ms - version_check_ms >= 5000) {
            // request Device Info every few seconds until we get a response
            version_check_ms = now_ms;

            // static uint32_t count = 1;
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG Requesting DeviceInfo %u", count++);

            const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO);
            UNUSED_RESULT(send_sbgMessage(uart, msg)); // don't care about any error, just retry at 1Hz

#if AP_COMPASS_ENABLED
        } else if (now_ms - send_MagData_ms >= 100) {
            send_MagData_ms = now_ms;

            if (!send_MagData(uart)) {
                // TODO: if it fails maybe we should figure out why and retry?
                // possible causes:
                // 1) uart == nullptr
                // 2) msg.len > sizeof(data)
                // 3) did not write all the bytes out the uart: zero/fail is treated same as packet_len != bytes_sent
                if (now_ms - send_mag_error_last_ms >= 5000) {
                    // throttle the error to no faster than 5Hz because if you get one error you'll likely get a lot
                    send_mag_error_last_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Error sending Mag data");
                }
            }
#endif // AP_COMPASS_ENABLED

#if AP_BARO_ENABLED || AP_AIRSPEED_ENABLED
        } else if (now_ms - send_AirData_ms >= 100) {
            send_AirData_ms = now_ms;

            if (!send_AirData(uart)) {
                // TODO: if it fails maybe we should figure out why and retry?
                // possible causes:
                // 1) uart == nullptr
                // 2) msg.len > sizeof(data)
                // 3) did not write all the bytes out the uart: zero/fail is treated same as packet_len != bytes_sent
                if (now_ms - send_air_error_last_ms >= 5000) {
                    // throttle the error to no faster than 5Hz because if you get one error you'll likely get a lot
                    send_air_error_last_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Error sending Air data");
                }
            }
#endif // AP_BARO_ENABLED || AP_AIRSPEED_ENABLED
        }
    } // while
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
// returns true if any data was found in the UART buffer which was then processed
bool AP_ExternalAHRS_SBG::check_uart()
{
    uint32_t nbytes = MIN(uart->available(), 512u);
    if (nbytes == 0) {
        return false;
    }
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }

        if (parse_byte(b, _inbound_state.msg, _inbound_state)) {
            handle_msg(_inbound_state.msg);
        }
    }
    return true;
}


bool AP_ExternalAHRS_SBG::send_sbgMessage(AP_HAL::UARTDriver *_uart, const sbgMessage &msg)
{
    if (_uart == nullptr || msg.len > ARRAY_SIZE(msg.data)) {
        // invalid _uart or msg.len is out of range
        return false;
    }

    const uint16_t buffer_len = SBG_PACKET_OVERHEAD + msg.len;
    uint8_t buffer[SBG_PACKET_OVERHEAD + SBG_PACKET_PAYLOAD_SIZE_MAX];

    buffer[0] = SBG_PACKET_SYNC1;
    buffer[1] = SBG_PACKET_SYNC2;
    buffer[2] = msg.msgid;
    buffer[3] = msg.msgclass;
    buffer[4] = msg.len & 0xFF; // LSB first
    buffer[5] = msg.len >> 8;

    for (uint16_t i=0; i<msg.len; i++) {
        buffer[6+i] = msg.data[i];
    }

    const uint16_t crc = crc16_ccitt_r((const uint8_t*)&msg, msg.len+4, 0, 0);

    buffer[buffer_len-3] = crc & 0xFF; // LSB First
    buffer[buffer_len-2] = crc >> 8;
    buffer[buffer_len-1] = SBG_PACKET_ETX;

    const uint32_t bytes_sent = _uart->write(buffer, buffer_len);
    _uart->flush();
    return (bytes_sent == buffer_len);
}

bool AP_ExternalAHRS_SBG::parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &inbound_state)
{
    switch (inbound_state.parser) {
    case SBG_PACKET_PARSE_STATE::SYNC1:
        inbound_state.parser = (data == SBG_PACKET_SYNC1) ? SBG_PACKET_PARSE_STATE::SYNC2 : SBG_PACKET_PARSE_STATE::SYNC1;
        break;

    case SBG_PACKET_PARSE_STATE::SYNC2:
        if (data == SBG_PACKET_SYNC2) {
            inbound_state.parser = SBG_PACKET_PARSE_STATE::MSG;
        } else {
            inbound_state.parser = (data == SBG_PACKET_SYNC1) ? SBG_PACKET_PARSE_STATE::SYNC2 : SBG_PACKET_PARSE_STATE::SYNC1;
        }
        break;

    case SBG_PACKET_PARSE_STATE::MSG:
        msg.msgid = data;
        inbound_state.parser = SBG_PACKET_PARSE_STATE::CLASS;
        break;

    case SBG_PACKET_PARSE_STATE::CLASS:
        msg.msgclass = data;
        inbound_state.parser = SBG_PACKET_PARSE_STATE::LEN1;
        break;

    case SBG_PACKET_PARSE_STATE::LEN1:
        msg.len = data;
        inbound_state.parser = SBG_PACKET_PARSE_STATE::LEN2;
        break;

    case SBG_PACKET_PARSE_STATE::LEN2:
        msg.len |= uint16_t(data) << 8;
        inbound_state.data_count = 0;
        if (msg.len > sizeof(msg.data)) {
            // Drop the payload, CRC and ETX. The next byte is then a possible SYNC1.
            inbound_state.data_count_skip = uint32_t(msg.len) + 3U;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::DROP_THIS_PACKET;
        } else {
            inbound_state.parser = (msg.len == 0) ? SBG_PACKET_PARSE_STATE::CRC1 : SBG_PACKET_PARSE_STATE::DATA;
        }
        break;

    case SBG_PACKET_PARSE_STATE::DATA:
        msg.data[inbound_state.data_count++] = data;
        if (inbound_state.data_count == msg.len) {
            inbound_state.parser = SBG_PACKET_PARSE_STATE::CRC1;
        }
        break;

    case SBG_PACKET_PARSE_STATE::CRC1:
        inbound_state.crc = data;
        inbound_state.parser = SBG_PACKET_PARSE_STATE::CRC2;
        break;

    case SBG_PACKET_PARSE_STATE::CRC2:
        inbound_state.crc |= uint16_t(data) << 8;
        inbound_state.crc_valid = crc16_ccitt_r((const uint8_t *)&msg, msg.len + 4, 0, 0) == inbound_state.crc;
        inbound_state.parser = SBG_PACKET_PARSE_STATE::ETX;
        break;

    case SBG_PACKET_PARSE_STATE::ETX: {
        const bool packet_valid = inbound_state.crc_valid && data == SBG_PACKET_ETX;
        inbound_state.parser = (data == SBG_PACKET_SYNC1 && !packet_valid) ?
            SBG_PACKET_PARSE_STATE::SYNC2 : SBG_PACKET_PARSE_STATE::SYNC1;
        return packet_valid;
    }

    case SBG_PACKET_PARSE_STATE::DROP_THIS_PACKET:
        if (inbound_state.data_count_skip > 0) {
            inbound_state.data_count_skip--;
        }
        if (inbound_state.data_count_skip == 0) {
            inbound_state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
        }
        break;
    }

    return false;
}

bool AP_ExternalAHRS_SBG::payload_length_valid(const uint8_t msgclass, const uint8_t msgid, const uint16_t payload_len)
{
    uint16_t minimum_length = 0;

    if (msgclass == SBG_ECOM_CLASS_LOG_CMD_0) {
        switch ((SbgEComCmd)msgid) {
        case SBG_ECOM_CMD_ACK:
            minimum_length = sizeof(SbgEComAck);
            break;
        case SBG_ECOM_CMD_INFO:
            minimum_length = sizeof(SbgEComDeviceInfo);
            break;
        default:
            return false;
        }
    } else if (msgclass == SBG_ECOM_CLASS_LOG_ECOM_1) {
        if (msgid != SBG_ECOM_LOG_FAST_IMU_DATA) {
            return false;
        }
        minimum_length = sizeof(SbgEComLogImuFastLegacy);
    } else if (msgclass == SBG_ECOM_CLASS_LOG_ECOM_0) {
        switch ((SbgEComLog)msgid) {
        case SBG_ECOM_LOG_STATUS:
            minimum_length = 22;
            break;
        case SBG_ECOM_LOG_UTC_TIME:
            minimum_length = 21;
            break;
        case SBG_ECOM_LOG_IMU_DATA:
            minimum_length = sizeof(SbgEComLogImuLegacy);
            break;
        case SBG_ECOM_LOG_MAG:
            minimum_length = sizeof(SbgEComLogMag);
            break;
        case SBG_ECOM_LOG_EKF_EULER:
            minimum_length = sizeof(SbgEComLogEkfEuler);
            break;
        case SBG_ECOM_LOG_EKF_QUAT:
            minimum_length = sizeof(SbgEComLogEkfQuat);
            break;
        case SBG_ECOM_LOG_EKF_NAV:
            minimum_length = sizeof(SbgEComLogEkfNav);
            break;
        case SBG_ECOM_LOG_GPS1_VEL:
        case SBG_ECOM_LOG_GPS2_VEL:
            minimum_length = sizeof(SbgEComLogGnssVel);
            break;
        case SBG_ECOM_LOG_GPS1_POS:
        case SBG_ECOM_LOG_GPS2_POS:
            minimum_length = 52;
            break;
        case SBG_ECOM_LOG_AIR_DATA:
            minimum_length = sizeof(SbgEComLogAirData);
            break;
        case SBG_ECOM_LOG_IMU_SHORT:
            minimum_length = sizeof(SbgEComLogImuShort);
            break;
        default:
            return false;
        }
    } else {
        return false;
    }

    return payload_len >= minimum_length;
}

void AP_ExternalAHRS_SBG::initialise_gnss_pos(SbgEComLogGnssPos &gnss_pos)
{
    memset(&gnss_pos, 0, sizeof(gnss_pos));
    gnss_pos.latitudeAccuracy = 9999.0f;
    gnss_pos.longitudeAccuracy = 9999.0f;
    gnss_pos.altitudeAccuracy = 9999.0f;
    gnss_pos.numSvUsed = UINT8_MAX;
    gnss_pos.numSvTracked = UINT8_MAX;
    gnss_pos.baseStationId = UINT16_MAX;
    gnss_pos.differentialAge = UINT16_MAX;
    gnss_pos.statusExt = uint32_t(SBG_ECOM_GNSS_IFM_STATUS_UNKNOWN) |
                         (uint32_t(SBG_ECOM_GNSS_SPOOFING_STATUS_UNKNOWN) << SBG_ECOM_GPS_POS_SPOOFING_SHIFT) |
                         (uint32_t(SBG_ECOM_GNSS_OSNMA_STATUS_DISABLED) << SBG_ECOM_GPS_POS_OSNMA_SHIFT);
    gnss_pos.upTime = UINT32_MAX;
}

void AP_ExternalAHRS_SBG::initialise_utc(SbgEComLogUtc &utc)
{
    memset(&utc, 0, sizeof(utc));
    utc.clkBiasStd = NAN;
    utc.clkSfErrorStd = NAN;
    utc.clkResidualError = NAN;
}

bool AP_ExternalAHRS_SBG::make_gps_week(const SbgEComLogUtc &utc_data, uint16_t &gps_week)
{
    static const uint8_t days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    const bool leap_year = (utc_data.year % 4U == 0U && utc_data.year % 100U != 0U) ||
                           utc_data.year % 400U == 0U;
    const uint8_t max_day = utc_data.month == 2 ? days_in_month[1] + uint8_t(leap_year) :
                                                  days_in_month[utc_data.month > 0 && utc_data.month <= 12 ? utc_data.month - 1 : 0];
    if (utc_data.year < 1980 || utc_data.month < 1 || utc_data.month > 12 ||
        utc_data.day < 1 || utc_data.day > max_day || utc_data.hour < 0 || utc_data.hour > 23 ||
        utc_data.minute < 0 || utc_data.minute > 59 || utc_data.second < 0 || utc_data.second > 60 ||
        utc_data.nanoSecond <= -1000000000 || utc_data.nanoSecond >= 1000000000 ||
        utc_data.gpsTimeOfWeek >= AP_MSEC_PER_WEEK) {
        return false;
    }

    const struct tm tm {
        .tm_sec = MIN(utc_data.second, 59),
        .tm_min = utc_data.minute,
        .tm_hour = utc_data.hour,
        .tm_mday = utc_data.day,
        .tm_mon = utc_data.month - 1,
        .tm_year = utc_data.year - 1900,
    };
    const int64_t unix_ms = int64_t(ap_mktime(&tm)) * AP_MSEC_PER_SEC +
                            utc_data.nanoSecond / 1000000 +
                            ((utc_data.second == 60) ? AP_MSEC_PER_SEC : 0);
    const int64_t gps_epoch_ms = unix_ms - int64_t(UNIX_OFFSET_MSEC);
    if (gps_epoch_ms < 0) {
        return false;
    }

    int64_t week = gps_epoch_ms / int64_t(AP_MSEC_PER_WEEK);
    const int64_t calendar_tow = gps_epoch_ms % int64_t(AP_MSEC_PER_WEEK);
    const int64_t reported_tow = utc_data.gpsTimeOfWeek;
    if (reported_tow - calendar_tow > int64_t(AP_MSEC_PER_WEEK / 2)) {
        week--;
    } else if (calendar_tow - reported_tow > int64_t(AP_MSEC_PER_WEEK / 2)) {
        week++;
    }
    if (week <= 0 || week > UINT16_MAX) {
        return false;
    }
    gps_week = uint16_t(week);
    return true;
}

bool AP_ExternalAHRS_SBG::extrapolate_gps_time(const TimeAnchor &anchor, const uint32_t sbg_timestamp,
                                               uint16_t &gps_week, uint32_t &tow_ms)
{
    if (!anchor.valid || anchor.week == 0 || anchor.tow_ms >= AP_MSEC_PER_WEEK) {
        gps_week = 0;
        tow_ms = 0;
        return false;
    }

    const int32_t delta_us = int32_t(sbg_timestamp - anchor.sbg_timestamp);
    int64_t extrapolated_tow = int64_t(anchor.tow_ms) + delta_us / 1000;
    int32_t week = anchor.week;
    while (extrapolated_tow < 0) {
        extrapolated_tow += AP_MSEC_PER_WEEK;
        week--;
    }
    while (extrapolated_tow >= int64_t(AP_MSEC_PER_WEEK)) {
        extrapolated_tow -= AP_MSEC_PER_WEEK;
        week++;
    }
    if (week <= 0 || week > UINT16_MAX) {
        gps_week = 0;
        tow_ms = 0;
        return false;
    }
    gps_week = uint16_t(week);
    tow_ms = uint32_t(extrapolated_tow);
    return true;
}

bool AP_ExternalAHRS_SBG::utc_anchor_valid(const SbgEComLogUtc &utc)
{
    const SbgEComClockStatus clock_status = sbgEComLogUtcGetClockStatus(utc.status);
    uint16_t week;
    return clock_status != SBG_ECOM_CLOCK_ERROR &&
           sbgEComLogUtcGetClockUtcStatus(utc.status) == SBG_ECOM_UTC_VALID &&
           make_gps_week(utc, week);
}

AP_ExternalAHRS_SBG::MessageFreshness AP_ExternalAHRS_SBG::message_freshness(
    const ReceiveTimes &times, const uint32_t now_ms)
{
    return {
        times.ekf_nav_ms != 0 && now_ms - times.ekf_nav_ms <= EKF_NAV_TIMEOUT_MS,
        times.gps_pos_ms != 0 && now_ms - times.gps_pos_ms <= GNSS_TIMEOUT_MS,
        times.gps_vel_ms != 0 && now_ms - times.gps_vel_ms <= GNSS_TIMEOUT_MS,
        times.utc_ms != 0 && now_ms - times.utc_ms <= UTC_TIMEOUT_MS,
        times.status_ms != 0 && now_ms - times.status_ms <= STATUS_TIMEOUT_MS,
    };
}

AP_ExternalAHRS_SBG::NavigationState AP_ExternalAHRS_SBG::navigation_state(
    const SbgEComLogEkfNav &ekf_nav, const bool nav_fresh, const bool disabled,
    float &horizontal_accuracy, float &speed_accuracy)
{
    horizontal_accuracy = NAN;
    speed_accuracy = NAN;
    if (disabled) {
        return NavigationState::ADMIN_DISABLED;
    }
    if (!nav_fresh) {
        return NavigationState::STALE;
    }

    const auto solution_mode = SbgEComSolutionMode(ekf_nav.status & SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK);
    if (solution_mode != SBG_ECOM_SOL_MODE_NAV_POSITION) {
        return NavigationState::NO_NAVIGATION;
    }
    if ((ekf_nav.status & (SBG_ECOM_SOL_POSITION_VALID | SBG_ECOM_SOL_VELOCITY_VALID)) !=
        (SBG_ECOM_SOL_POSITION_VALID | SBG_ECOM_SOL_VELOCITY_VALID)) {
        return NavigationState::INVALID_NAVIGATION;
    }

    for (uint8_t i = 0; i < 3; i++) {
        if (!isfinite(ekf_nav.velocity[i]) || !isfinite(ekf_nav.velocityStdDev[i]) ||
            is_negative(ekf_nav.velocityStdDev[i]) || !isfinite(ekf_nav.position[i]) ||
            !isfinite(ekf_nav.positionStdDev[i]) || is_negative(ekf_nav.positionStdDev[i])) {
            return NavigationState::INVALID_NAVIGATION;
        }
    }
    if (fabs(ekf_nav.position[0]) > 90.0 || fabs(ekf_nav.position[1]) > 180.0 ||
        fabs(ekf_nav.position[2]) > 1000000.0) {
        return NavigationState::INVALID_NAVIGATION;
    }

    horizontal_accuracy = safe_sqrt(sq(ekf_nav.positionStdDev[0]) + sq(ekf_nav.positionStdDev[1]));
    speed_accuracy = safe_sqrt(sq(ekf_nav.velocityStdDev[0]) + sq(ekf_nav.velocityStdDev[1]) +
                               sq(ekf_nav.velocityStdDev[2]));
    if (!isfinite(horizontal_accuracy) || horizontal_accuracy >= 100.0f || !isfinite(speed_accuracy)) {
        return NavigationState::INVALID_NAVIGATION;
    }
    return (ekf_nav.status & SBG_ECOM_SOL_GPS1_POS_USED) ? NavigationState::GNSS_AIDED : NavigationState::INERTIAL;
}

AP_GPS_FixType AP_ExternalAHRS_SBG::accuracy_to_fix(const float horizontal_accuracy)
{
    if (!isfinite(horizontal_accuracy) || is_negative(horizontal_accuracy) || horizontal_accuracy >= 100.0f) {
        return AP_GPS_FixType::NONE;
    }
    if (horizontal_accuracy < 0.10f) {
        return AP_GPS_FixType::RTK_FIXED;
    }
    if (horizontal_accuracy < 0.30f) {
        return AP_GPS_FixType::RTK_FLOAT;
    }
    if (horizontal_accuracy < 1.20f) {
        return AP_GPS_FixType::DGPS;
    }
    return AP_GPS_FixType::FIX_3D;
}

AP_GPS_FixType AP_ExternalAHRS_SBG::navigation_fix(const NavigationState nav_state,
                                                   const SbgEComLogEkfNav &ekf_nav,
                                                   const SbgEComLogGnssPos &gnss_pos,
                                                   const bool gnss_pos_fresh,
                                                   const bool gnss_vel_fresh,
                                                   const float horizontal_accuracy)
{
    if (nav_state != NavigationState::INERTIAL && nav_state != NavigationState::GNSS_AIDED) {
        return AP_GPS_FixType::NONE;
    }

    const uint8_t ifm = (gnss_pos.statusExt >> SBG_ECOM_GPS_POS_IFM_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t spoofing = (gnss_pos.statusExt >> SBG_ECOM_GPS_POS_SPOOFING_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t osnma = (gnss_pos.statusExt >> SBG_ECOM_GPS_POS_OSNMA_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const bool unsafe_gnss = ifm == SBG_ECOM_GNSS_IFM_STATUS_CRITICAL ||
                             spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_SINGLE ||
                             spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_MULTIPLE ||
                             osnma == SBG_ECOM_GNSS_OSNMA_STATUS_SPOOFED;
    const uint8_t position_status = (gnss_pos.status >> SBG_ECOM_GPS_POS_STATUS_SHIFT) & SBG_ECOM_GPS_POS_STATUS_MASK;
    const AP_GPS_FixType raw_fix = gps_position_type_to_fix(gnss_pos.status);
    const bool trusted_classification = nav_state == NavigationState::GNSS_AIDED && gnss_pos_fresh && gnss_vel_fresh &&
                                        (ekf_nav.status & SBG_ECOM_SOL_GPS1_POS_USED) &&
                                        position_status == SBG_ECOM_GPS_POS_STATUS_SOL_COMPUTED &&
                                        raw_fix >= AP_GPS_FixType::FIX_3D && !unsafe_gnss;
    if (!trusted_classification) {
        return AP_GPS_FixType::FIX_3D;
    }
    return MIN(raw_fix, accuracy_to_fix(horizontal_accuracy));
}

void AP_ExternalAHRS_SBG::handle_msg(const sbgMessage &msg)
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool valid_class1_msg = ((SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_ECOM_1 && (_SbgEComLog1MsgId)msg.msgid == SBG_ECOM_LOG_FAST_IMU_DATA);

    if (!payload_length_valid(msg.msgclass, msg.msgid, msg.len)) {
        return;
    }

    if ((SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_CMD_0) {
        switch ((SbgEComCmd)msg.msgid) {
            case SBG_ECOM_CMD_ACK: // 0
                break;

            case SBG_ECOM_CMD_INFO: // 4
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.deviceInfo, sizeof(cached.sbg.deviceInfo), msg.data, msg.len);

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: %s", cached.sbg.deviceInfo.productCode);
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: %u, %u, %u, %u, %u, %u, %u: %u.%u.%u",
                //                 (unsigned)cached.sbg.deviceInfo.serialNumber,
                //                 (unsigned)cached.sbg.deviceInfo.calibationRev,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationYear,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationMonth,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationDay,
                //                 (unsigned)cached.sbg.deviceInfo.hardwareRev,
                //                 (unsigned)cached.sbg.deviceInfo.firmwareRev,
                //                 (cached.sbg.deviceInfo.firmwareRev >> 22) & 0x3F,
                //                 (cached.sbg.deviceInfo.firmwareRev >> 16) & 0x3F,
                //                 cached.sbg.deviceInfo.firmwareRev & 0xFFFF

                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Serial Number: %u",(unsigned)cached.sbg.deviceInfo.serialNumber);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Version HW v%u.%u   FW v%u.%u.%u",
                                (unsigned)(cached.sbg.deviceInfo.hardwareRev >> 24) & 0x00FF,
                                (unsigned)(cached.sbg.deviceInfo.hardwareRev >> 16) & 0x00FF,

                                (unsigned) (cached.sbg.deviceInfo.firmwareRev >> 22) & 0x003F,
                                (unsigned)(cached.sbg.deviceInfo.firmwareRev >> 16) & 0x003F,
                                (unsigned)cached.sbg.deviceInfo.firmwareRev & 0xFFFF);
                break;

            default:
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Unknown ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
                break;
        }
        return;
    }



    if (((SbgEComClass)msg.msgclass != SBG_ECOM_CLASS_LOG_ECOM_0) && !valid_class1_msg) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Unknown ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
        return;
    }

    const bool use_ekf_as_gnss = option_is_set(AP_ExternalAHRS::OPTIONS::SBG_EKF_AS_GNSS);

    bool updated_gps = false;
    bool updated_baro = false;
    bool updated_ins = false;
    bool updated_mag = false;
    bool updated_airspeed = false;

    {
        WITH_SEMAPHORE(state.sem);

        switch (msg.msgid) {  // (_SbgEComLog)
            case SBG_ECOM_LOG_FAST_IMU_DATA: // 0
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuFastLegacy, sizeof(cached.sbg.imuFastLegacy), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.imuFastLegacy.accelerometers[0], cached.sbg.imuFastLegacy.accelerometers[1], cached.sbg.imuFastLegacy.accelerometers[2]);
                state.gyro = Vector3f(cached.sbg.imuFastLegacy.gyroscopes[0], cached.sbg.imuFastLegacy.gyroscopes[1], cached.sbg.imuFastLegacy.gyroscopes[2]);
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_UTC_TIME: // 2
                initialise_utc(cached.sbg.utc);
                memcpy(&cached.sbg.utc, msg.data, MIN(sizeof(cached.sbg.utc), msg.len));
                received.utc_ms = now_ms;

                time_anchor.valid = utc_anchor_valid(cached.sbg.utc) &&
                                    make_gps_week(cached.sbg.utc, time_anchor.week);
                if (time_anchor.valid) {
                    time_anchor.tow_ms = cached.sbg.utc.gpsTimeOfWeek;
                    time_anchor.sbg_timestamp = cached.sbg.utc.timeStamp;
                }

#if AP_RTC_ENABLED
                if (time_anchor.valid &&
                    sbgEComLogUtcGetClockStatus(cached.sbg.utc.status) == SBG_ECOM_CLOCK_VALID &&
                    (cached.sbg.utc.status & SBG_ECOM_CLOCK_UTC_IS_ACCURATE)) {
                    const uint32_t utc_epoch_sec = AP::rtc().date_fields_to_clock_s(
                        cached.sbg.utc.year,
                        cached.sbg.utc.month - 1,
                        cached.sbg.utc.day,
                        cached.sbg.utc.hour,
                        cached.sbg.utc.minute,
                        MIN(cached.sbg.utc.second, 59));

                    const int64_t utc_epoch_usec = int64_t(utc_epoch_sec) * 1000000LL +
                                                   cached.sbg.utc.nanoSecond / 1000 +
                                                   (cached.sbg.utc.second == 60 ? 1000000LL : 0LL);
                    if (utc_epoch_usec > 0) {
                        AP::rtc().set_utc_usec(uint64_t(utc_epoch_usec), AP_RTC::SOURCE_GPS);
                    }
                }
#endif // AP_RTC_ENABLED

                if (!use_ekf_as_gnss && time_anchor.valid) {
                    cached.sensors.gps_data.ms_tow = time_anchor.tow_ms;
                    cached.sensors.gps_data.gps_week = time_anchor.week;
                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_STATUS:
                memset(&cached.sbg.status, 0, sizeof(cached.sbg.status));
                cached.sbg.status.cpuUsage = UINT8_MAX;
                memcpy(&cached.sbg.status, msg.data, MIN(sizeof(cached.sbg.status), msg.len));
                if (msg.len < 26) {
                    cached.sbg.status.uptime = 0;
                }
                received.status_ms = now_ms;
                break;
            
            case SBG_ECOM_LOG_IMU_SHORT: // 44
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuShort, sizeof(cached.sbg.imuShort), msg.data, msg.len);

                {
                    Vector3f temp;
                    /*!< X, Y, Z delta velocity. Unit is 1048576 LSB for 1 m.s^-2. */
                    temp = Vector3f(cached.sbg.imuShort.deltaVelocity[0], cached.sbg.imuShort.deltaVelocity[1], cached.sbg.imuShort.deltaVelocity[2]);
                    state.accel = temp / SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;

                    /*!< X, Y, Z delta angle. Unit is either 67108864 LSB for 1 rad.s^-1 (standard) or 12304174 LSB for 1 rad.s^-1 (high range), managed automatically. */
                    temp = Vector3f(cached.sbg.imuShort.deltaAngle[0], cached.sbg.imuShort.deltaAngle[1], cached.sbg.imuShort.deltaAngle[2]);
                    const float scaleFactor = (cached.sbg.imuShort.status & SBG_ECOM_IMU_GYROS_USE_HIGH_SCALE) ? SBG_ECOM_LOG_IMU_GYRO_SCALE_HIGH : SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;
                    state.gyro = temp / scaleFactor;
                }
                cached.sensors.ins_data.temperature = (cached.sbg.imuShort.temperature / SBG_ECOM_LOG_IMU_TEMP_SCALE_STD);
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_IMU_DATA: // 3
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuLegacy, sizeof(cached.sbg.imuLegacy), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.imuLegacy.accelerometers[0], cached.sbg.imuLegacy.accelerometers[1], cached.sbg.imuLegacy.accelerometers[2]);
                state.gyro = Vector3f(cached.sbg.imuLegacy.gyroscopes[0], cached.sbg.imuLegacy.gyroscopes[1], cached.sbg.imuLegacy.gyroscopes[2]);
                cached.sensors.ins_data.temperature = cached.sbg.imuLegacy.temperature;
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_MAG: // 4
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.mag, sizeof(cached.sbg.mag), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.mag.accelerometers[0], cached.sbg.mag.accelerometers[1], cached.sbg.mag.accelerometers[2]);
                updated_ins = true;

                cached.sensors.mag_data.field = Vector3f(cached.sbg.mag.magnetometers[0], cached.sbg.mag.magnetometers[1], cached.sbg.mag.magnetometers[2]) * 1000.0f;
                updated_mag = true;
                break;

            case SBG_ECOM_LOG_EKF_EULER: // 6
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfEuler, sizeof(cached.sbg.ekfEuler), msg.data, msg.len);

                // float	euler[3];				/*!< Roll, Pitch and Yaw angles in rad. */
                // float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
                state.quat.from_euler(cached.sbg.ekfEuler.euler[0], cached.sbg.ekfEuler.euler[1], cached.sbg.ekfEuler.euler[2]);
                state.have_quaternion = true;
                break;
            
            case SBG_ECOM_LOG_EKF_QUAT: // 7
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfQuat, sizeof(cached.sbg.ekfQuat), msg.data, msg.len);

                // float	quaternion[4];			/*!< Orientation quaternion stored in W, X, Y, Z form. */
                // float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
                memcpy(&state.quat, cached.sbg.ekfQuat.quaternion, sizeof(state.quat));
                state.have_quaternion = true;
                break;

            case SBG_ECOM_LOG_EKF_NAV: // 8
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfNav, sizeof(cached.sbg.ekfNav), msg.data, msg.len);
                received.ekf_nav_ms = now_ms;

                {
                    float horizontal_accuracy;
                    float speed_accuracy;
                    const NavigationState nav_state = navigation_state(cached.sbg.ekfNav, true, false,
                                                                       horizontal_accuracy, speed_accuracy);
                    const bool valid_navigation = nav_state == NavigationState::INERTIAL ||
                                                  nav_state == NavigationState::GNSS_AIDED;
                    state.have_velocity = valid_navigation;
                    state.have_location = valid_navigation;
                    if (valid_navigation) {
                        state.velocity = Vector3f(cached.sbg.ekfNav.velocity[0], cached.sbg.ekfNav.velocity[1], cached.sbg.ekfNav.velocity[2]);
                        state.location = Location(cached.sbg.ekfNav.position[0] * 1e7,
                                                  cached.sbg.ekfNav.position[1] * 1e7,
                                                  cached.sbg.ekfNav.position[2] * 1e2,
                                                  Location::AltFrame::ABSOLUTE);
                        state.last_location_update_us = AP_HAL::micros();
                        if (!state.have_origin && cached.sensors.gps_data.fix_type >= AP_GPS_FixType::FIX_3D) {
                            state.origin = state.location;
                            state.have_origin = true;
                        }
                    }
                }
                break;

            case SBG_ECOM_LOG_GPS1_VEL: // 13
            case SBG_ECOM_LOG_GPS2_VEL: // 16
                if (use_ekf_as_gnss && msg.msgid == SBG_ECOM_LOG_GPS2_VEL) {
                    break;
                }
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.gnssVel, sizeof(cached.sbg.gnssVel), msg.data, msg.len);
                received.gps_vel_ms = now_ms;

                if (!use_ekf_as_gnss) {
                    cached.sensors.gps_data.ms_tow = cached.sbg.gnssVel.timeOfWeek;
                    cached.sensors.gps_data.ned_vel_north = cached.sbg.gnssVel.velocity[0];
                    cached.sensors.gps_data.ned_vel_east = cached.sbg.gnssVel.velocity[1];
                    cached.sensors.gps_data.ned_vel_down = cached.sbg.gnssVel.velocity[2];
                    cached.sensors.gps_data.horizontal_vel_accuracy = Vector2f(cached.sbg.gnssVel.velocityAcc[0], cached.sbg.gnssVel.velocityAcc[1]).length();
                    cached.sensors.gps_data.speed_accuracy = Vector3f(cached.sbg.gnssVel.velocityAcc[0], cached.sbg.gnssVel.velocityAcc[1], cached.sbg.gnssVel.velocityAcc[2]).length();
                    cached.sensors.gps_data.have_speed_accuracy = isfinite(cached.sensors.gps_data.speed_accuracy);
                    cached.sensors.gps_data.have_vertical_velocity = true;
                    // unused - cached.sbg.gnssVel.course
                    // unused - cached.sbg.gnssVel.courseAcc
                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_GPS1_POS: // 14
            case SBG_ECOM_LOG_GPS2_POS: // 17
                if (use_ekf_as_gnss && msg.msgid == SBG_ECOM_LOG_GPS2_POS) {
                    break;
                }
                initialise_gnss_pos(cached.sbg.gnssPos);
                memcpy(&cached.sbg.gnssPos, msg.data, MIN(sizeof(cached.sbg.gnssPos), msg.len));
                received.gps_pos_ms = now_ms;

                if (!use_ekf_as_gnss) {
                    cached.sensors.gps_data.ms_tow = cached.sbg.gnssPos.timeOfWeek;
                    cached.sensors.gps_data.latitude = cached.sbg.gnssPos.latitude * 1E7;
                    cached.sensors.gps_data.longitude = cached.sbg.gnssPos.longitude * 1E7;
                    cached.sensors.gps_data.msl_altitude = cached.sbg.gnssPos.altitude * 100;
                    // SBG reports ellipsoid minus MSL; AP_GPS uses MSL minus ellipsoid.
                    cached.sensors.gps_data.undulation = -cached.sbg.gnssPos.undulation;
                    cached.sensors.gps_data.have_undulation = isfinite(cached.sbg.gnssPos.undulation);
                    cached.sensors.gps_data.horizontal_pos_accuracy = Vector2f(cached.sbg.gnssPos.latitudeAccuracy, cached.sbg.gnssPos.longitudeAccuracy).length();
                    cached.sensors.gps_data.hdop = cached.sensors.gps_data.horizontal_pos_accuracy;
                    cached.sensors.gps_data.vertical_pos_accuracy = cached.sbg.gnssPos.altitudeAccuracy;
                    cached.sensors.gps_data.vdop =  cached.sensors.gps_data.vertical_pos_accuracy;
                    cached.sensors.gps_data.have_horizontal_accuracy = isfinite(cached.sensors.gps_data.horizontal_pos_accuracy);
                    cached.sensors.gps_data.have_vertical_accuracy = isfinite(cached.sensors.gps_data.vertical_pos_accuracy);
                    cached.sensors.gps_data.satellites_in_view = cached.sbg.gnssPos.numSvUsed == UINT8_MAX ? 0 : cached.sbg.gnssPos.numSvUsed;
                    const uint8_t position_status = (cached.sbg.gnssPos.status >> SBG_ECOM_GPS_POS_STATUS_SHIFT) & SBG_ECOM_GPS_POS_STATUS_MASK;
                    cached.sensors.gps_data.fix_type = position_status == SBG_ECOM_GPS_POS_STATUS_SOL_COMPUTED ?
                        gps_position_type_to_fix(cached.sbg.gnssPos.status) : AP_GPS_FixType::NONE;
                    if (cached.sensors.gps_data.fix_type >= AP_GPS_FixType::RTK_FLOAT) {
                        cached.sensors.gps_data.rtk_age_ms = cached.sbg.gnssPos.differentialAge == UINT16_MAX ?
                            UINT32_MAX : uint32_t(cached.sbg.gnssPos.differentialAge) * 10U;
                        cached.sensors.gps_data.rtk_num_sats = cached.sensors.gps_data.satellites_in_view;
                    } else {
                        cached.sensors.gps_data.rtk_age_ms = 0;
                        cached.sensors.gps_data.rtk_num_sats = 0;
                    }
                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_AIR_DATA: // 36
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.airData, sizeof(cached.sbg.airData), msg.data, msg.len);
                
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID) {
                    cached.sensors.baro_data.pressure_pa = cached.sbg.airData.pressureAbs;
                    updated_baro = true;
                }
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_ALTITUDE_VALID) {
                    cached.sensors.baro_height = cached.sbg.airData.altitude;
                    updated_baro = true;
                }
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID) {
                    cached.sensors.airspeed_data.differential_pressure = cached.sbg.airData.pressureDiff;
                    updated_airspeed = true;
                }
                // if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_AIRPSEED_VALID) {
                //     // we don't accept airspeed directly, we compute it ourselves in AP_Airspeed via diff pressure
                // }
                
                if ((cached.sbg.airData.status & SBG_ECOM_AIR_DATA_TEMPERATURE_VALID) && (updated_baro || updated_airspeed)) {
                    cached.sensors.airspeed_data.temperature = cached.sbg.airData.airTemperature;
                    cached.sensors.baro_data.temperature = cached.sbg.airData.airTemperature;
                }
                break;

            default:
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: unhandled ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
                return;
        } // switch msgid
    } // semaphore

#if AP_GPS_ENABLED
    if (updated_gps) {
        cached.sensors.gps_ms = now_ms;
        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(cached.sensors.gps_data, instance);
        }
    }
#else
    (void)updated_gps;
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (updated_mag) {
        cached.sensors.mag_ms = now_ms;
        AP::compass().handle_external(cached.sensors.mag_data);
    }
#else
    (void)updated_mag;
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (updated_baro) {
        cached.sensors.baro_ms = now_ms;
        cached.sensors.baro_data.instance = 0;
        AP::baro().handle_external(cached.sensors.baro_data);
    }
#else
    (void)updated_baro;
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    if (updated_airspeed && AP::airspeed() != nullptr) {
        cached.sensors.airspeed_ms = now_ms;
        AP::airspeed()->handle_external(cached.sensors.airspeed_data);
    }
#else
    (void)updated_airspeed;
#endif


    if (updated_ins) {
        cached.sensors.ins_data.accel = state.accel;
        cached.sensors.ins_data.gyro = state.gyro;
        cached.sensors.ins_ms = now_ms;
        AP::ins().handle_external(cached.sensors.ins_data);
    }

    last_received_ms = now_ms;

    if (use_ekf_as_gnss) {
        publish_ekf_gps(now_ms);
    }
}

void AP_ExternalAHRS_SBG::safe_copy_msg_to_object(uint8_t* dest, const uint16_t dest_len, const uint8_t* src, const uint16_t src_len)
{
    // To allow for future changes of the SBG protocol the protocol allows extending messages
    // which can be detected by a length mismatch, usually longer. To allow for this you assume unused data is zero
    // but instead of zeroing the packet every time before populating it it's best to only zero when necessary.
    if (dest_len != src_len) {
        memset(dest, 0, dest_len);
    }
    memcpy(dest, src, MIN(dest_len,src_len));
}

AP_GPS_FixType AP_ExternalAHRS_SBG::gps_position_type_to_fix(const uint32_t gpsPosStatus)
{
    const uint32_t fix = (gpsPosStatus >> SBG_ECOM_GPS_POS_TYPE_SHIFT) & SBG_ECOM_GPS_POS_TYPE_MASK;
    switch ((SbgEComGpsPosType)fix) {
        case SBG_ECOM_POS_NO_SOLUTION:      /*!< No valid solution available. */
        case SBG_ECOM_POS_UNKNOWN_TYPE:     /*!< An unknown solution type has been computed. */
            return AP_GPS_FixType::NONE;

        case SBG_ECOM_POS_SINGLE:           /*!< Single point solution position. */
        case SBG_ECOM_POS_FIXED:            /*!< Fixed location solution position. */
            return AP_GPS_FixType::FIX_3D;

        case SBG_ECOM_POS_PSRDIFF:          /*!< Standard Pseudorange Differential Solution (DGPS). */
        case SBG_ECOM_POS_SBAS:             /*!< SBAS satellite used for differential corrections. */
            return AP_GPS_FixType::DGPS;

        case SBG_ECOM_POS_RTK_FLOAT:        /*!< Floating RTK ambiguity solution (20 cms RTK). */
        case SBG_ECOM_POS_PPP_FLOAT:        /*!< Precise Point Positioning with float ambiguities. */
        case SBG_ECOM_POS_OMNISTAR:         /*!< Omnistar VBS Position (L1 sub-meter). */
            return AP_GPS_FixType::RTK_FLOAT;

        case SBG_ECOM_POS_RTK_INT:          /*!< Integer RTK ambiguity solution (2 cms RTK). */
        case SBG_ECOM_POS_PPP_INT:          /*!< Precise Point Positioning with fixed ambiguities. */
            return AP_GPS_FixType::RTK_FIXED;
    }
    return AP_GPS_FixType::NONE;
}

void AP_ExternalAHRS_SBG::publish_ekf_gps(const uint32_t now_ms)
{
#if AP_GPS_ENABLED
    uint8_t instance;
    if (!AP::gps().get_first_external_instance(instance)) {
        return;
    }
    const uint16_t publish_period_ms = constrain_int16(AP::gps().get_rate_ms(instance), 50, 200);
    if (gps_has_published && now_ms - last_gps_publish_ms < publish_period_ms) {
        return;
    }
    gps_has_published = true;
    last_gps_publish_ms = now_ms;

    AP_ExternalAHRS::gps_data_message_t gps_data {};
    NavigationState nav_state;
    AP_GPS_FixType fix_type;
    float horizontal_accuracy;
    float speed_accuracy;
    float vertical_accuracy = NAN;
    uint32_t differential_age_ms = 0;
    bool nav_stale;
    bool status_stale;
    bool utc_valid;

    {
        WITH_SEMAPHORE(state.sem);
        const MessageFreshness freshness = message_freshness(received, now_ms);
        nav_stale = !freshness.ekf_nav;
        status_stale = !freshness.status;
        utc_valid = time_anchor.valid && freshness.utc;

        nav_state = navigation_state(cached.sbg.ekfNav, !nav_stale, gnss_is_disabled(),
                                     horizontal_accuracy, speed_accuracy);
        fix_type = navigation_fix(nav_state, cached.sbg.ekfNav, cached.sbg.gnssPos,
                                  freshness.gps_pos, freshness.gps_vel, horizontal_accuracy);

        gps_data = cached.sensors.gps_data;
        gps_data.fix_type = fix_type;
        gps_data.gps_week = 0;
        gps_data.ms_tow = 0;
        gps_data.satellites_in_view = 0;
        gps_data.rtk_age_ms = 0;
        gps_data.rtk_num_sats = 0;
        gps_data.have_horizontal_accuracy = false;
        gps_data.have_vertical_accuracy = false;
        gps_data.have_speed_accuracy = false;
        gps_data.have_vertical_velocity = false;
        gps_data.have_undulation = false;
        gps_data.horizontal_pos_accuracy = NAN;
        gps_data.vertical_pos_accuracy = NAN;
        gps_data.horizontal_vel_accuracy = NAN;
        gps_data.speed_accuracy = NAN;
        gps_data.hdop = NAN;
        gps_data.vdop = NAN;
        gps_data.undulation = NAN;

        const bool valid_navigation = nav_state == NavigationState::INERTIAL ||
                                      nav_state == NavigationState::GNSS_AIDED;
        if (valid_navigation) {
            gps_data.latitude = cached.sbg.ekfNav.position[0] * 1.0e7;
            gps_data.longitude = cached.sbg.ekfNav.position[1] * 1.0e7;
            gps_data.msl_altitude = cached.sbg.ekfNav.position[2] * 100.0;
            gps_data.ned_vel_north = cached.sbg.ekfNav.velocity[0];
            gps_data.ned_vel_east = cached.sbg.ekfNav.velocity[1];
            gps_data.ned_vel_down = cached.sbg.ekfNav.velocity[2];

            vertical_accuracy = cached.sbg.ekfNav.positionStdDev[2];
            gps_data.horizontal_pos_accuracy = horizontal_accuracy;
            gps_data.vertical_pos_accuracy = vertical_accuracy;
            gps_data.horizontal_vel_accuracy = Vector2f(cached.sbg.ekfNav.velocityStdDev[0],
                                                         cached.sbg.ekfNav.velocityStdDev[1]).length();
            gps_data.speed_accuracy = speed_accuracy;
            gps_data.hdop = horizontal_accuracy;
            gps_data.vdop = vertical_accuracy;
            gps_data.have_horizontal_accuracy = true;
            gps_data.have_vertical_accuracy = true;
            gps_data.have_speed_accuracy = true;
            gps_data.have_vertical_velocity = true;
            // SBG reports ellipsoid minus MSL; AP_GPS uses MSL minus ellipsoid.
            gps_data.undulation = -cached.sbg.ekfNav.undulation;
            gps_data.have_undulation = isfinite(cached.sbg.ekfNav.undulation);

            if (utc_valid) {
                extrapolate_gps_time(time_anchor, cached.sbg.ekfNav.timeStamp,
                                     gps_data.gps_week, gps_data.ms_tow);
            }
        }

        if (freshness.gps_pos && cached.sbg.gnssPos.numSvUsed != UINT8_MAX) {
            gps_data.satellites_in_view = cached.sbg.gnssPos.numSvUsed;
        }
        if (freshness.gps_pos) {
            differential_age_ms = cached.sbg.gnssPos.differentialAge == UINT16_MAX ?
                UINT32_MAX : uint32_t(cached.sbg.gnssPos.differentialAge) * 10U;
        }
        if (fix_type >= AP_GPS_FixType::RTK_FLOAT) {
            gps_data.rtk_age_ms = differential_age_ms;
            gps_data.rtk_num_sats = gps_data.satellites_in_view;
        }

        cached.sensors.gps_data = gps_data;
        cached.sensors.gps_ms = now_ms;
    }

    update_transition_messages(now_ms, nav_state, fix_type, nav_stale, status_stale, utc_valid);
    write_sbg_log(nav_state, fix_type, horizontal_accuracy, vertical_accuracy,
                  speed_accuracy, differential_age_ms);
    AP::gps().handle_external(gps_data, instance);
#else
    (void)now_ms;
#endif
}

void AP_ExternalAHRS_SBG::update_transition_messages(const uint32_t now_ms,
                                                     const NavigationState nav_state,
                                                     const AP_GPS_FixType fix_type,
                                                     const bool nav_stale,
                                                     const bool status_stale,
                                                     const bool utc_valid)
{
    const uint8_t ekf_mode = cached.sbg.ekfNav.status & SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK;
    const bool zupt = cached.sbg.ekfNav.status & SBG_ECOM_SOL_ZUPT_USED;
    const uint8_t ifm = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_IFM_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t spoofing = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_SPOOFING_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t osnma = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_OSNMA_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const bool first_update = previous_ekf_mode == UINT8_MAX;

    if (!nav_stale && ekf_mode != previous_ekf_mode &&
        (mode_message_ms == 0 || now_ms - mode_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: EKF mode %u", unsigned(ekf_mode));
        mode_message_ms = now_ms;
    }
    if (!nav_stale) {
        previous_ekf_mode = ekf_mode;
    }

    if (have_previous_nav_state && nav_state != previous_nav_state &&
        (nav_message_ms == 0 || now_ms - nav_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        if (nav_state == NavigationState::INERTIAL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: inertial navigation");
        } else if (nav_state == NavigationState::GNSS_AIDED) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: GNSS aiding restored (%u)", unsigned(fix_type));
        }
        nav_message_ms = now_ms;
    }
    previous_nav_state = nav_state;
    have_previous_nav_state = true;

    if (!first_update && zupt != previous_zupt &&
        (zupt_message_ms == 0 || now_ms - zupt_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, zupt ? "SBG: ZUPT active" : "SBG: ZUPT cleared");
        zupt_message_ms = now_ms;
    }
    previous_zupt = zupt;

    if (previous_ifm != UINT8_MAX && ifm != previous_ifm &&
        (interference_message_ms == 0 || now_ms - interference_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        if (ifm == SBG_ECOM_GNSS_IFM_STATUS_CRITICAL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: GNSS interference critical");
        } else if (ifm == SBG_ECOM_GNSS_IFM_STATUS_MITIGATED) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: GNSS interference mitigated");
        } else if (previous_ifm == SBG_ECOM_GNSS_IFM_STATUS_CRITICAL ||
                   previous_ifm == SBG_ECOM_GNSS_IFM_STATUS_MITIGATED) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: GNSS interference cleared");
        }
        interference_message_ms = now_ms;
    }
    previous_ifm = ifm;

    if (previous_spoofing != UINT8_MAX && (spoofing != previous_spoofing || osnma != previous_osnma) &&
        (spoofing_message_ms == 0 || now_ms - spoofing_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        if (osnma == SBG_ECOM_GNSS_OSNMA_STATUS_SPOOFED) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: GNSS OSNMA spoofing");
        } else if (spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_MULTIPLE) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: GNSS spoofing confirmed");
        } else if (spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_SINGLE) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: GNSS spoofing probable");
        } else if (previous_spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_SINGLE ||
                   previous_spoofing == SBG_ECOM_GNSS_SPOOFING_STATUS_MULTIPLE ||
                   previous_osnma == SBG_ECOM_GNSS_OSNMA_STATUS_SPOOFED) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: GNSS spoofing cleared");
        }
        spoofing_message_ms = now_ms;
    }
    previous_spoofing = spoofing;
    previous_osnma = osnma;

    if (!first_update && utc_valid != previous_utc_valid &&
        (utc_message_ms == 0 || now_ms - utc_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        GCS_SEND_TEXT(utc_valid ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                      utc_valid ? "SBG: UTC valid" : "SBG: UTC invalid");
        utc_message_ms = now_ms;
    }
    previous_utc_valid = utc_valid;

    if ((!first_update && (nav_stale != previous_nav_stale || status_stale != previous_status_stale)) &&
        (stale_message_ms == 0 || now_ms - stale_message_ms >= TRANSITION_MESSAGE_INTERVAL_MS)) {
        if (nav_stale && status_stale) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: EKF NAV/STATUS stale");
        } else if (nav_stale) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: EKF NAV stale");
        } else if (status_stale) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SBG: STATUS stale");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: required logs restored");
        }
        stale_message_ms = now_ms;
    }
    previous_nav_stale = nav_stale;
    previous_status_stale = status_stale;
}

void AP_ExternalAHRS_SBG::write_sbg_log(const NavigationState nav_state,
                                        const AP_GPS_FixType fix_type,
                                        const float horizontal_accuracy,
                                        const float vertical_accuracy,
                                        const float speed_accuracy,
                                        const uint32_t differential_age_ms) const
{
#if HAL_LOGGING_ENABLED
    const uint8_t ifm = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_IFM_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t spoofing = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_SPOOFING_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t osnma = (cached.sbg.gnssPos.statusExt >> SBG_ECOM_GPS_POS_OSNMA_SHIFT) & SBG_ECOM_GPS_POS_EXT_STATUS_MASK;
    const uint8_t sv_used = cached.sbg.gnssPos.numSvUsed == UINT8_MAX ? 0 : cached.sbg.gnssPos.numSvUsed;
    const uint8_t sv_tracked = cached.sbg.gnssPos.numSvTracked == UINT8_MAX ? 0 : cached.sbg.gnssPos.numSvTracked;
    const uint8_t gnss_used = uint8_t(bool(cached.sbg.ekfNav.status & SBG_ECOM_SOL_GPS1_POS_USED)) |
                              (uint8_t(bool(cached.sbg.ekfNav.status & SBG_ECOM_SOL_GPS1_VEL_USED)) << 1);
    const uint16_t security = uint16_t(ifm) |
                              (uint16_t(spoofing) << 4) |
                              (uint16_t(osnma) << 8);
    const uint16_t satellites = uint16_t(sv_used) | (uint16_t(sv_tracked) << 8);
    const uint8_t flags = uint8_t(bool(cached.sbg.ekfNav.status & SBG_ECOM_SOL_ZUPT_USED)) |
                          (uint8_t(bool(cached.sbg.ekfNav.status & SBG_ECOM_SOL_ALIGN_VALID)) << 1) |
                          (gnss_used << 2);

    // FMT records support at most 16 fields and 64 label characters. Pack
    // related diagnostics so this high-rate record remains self describing.
    AP::logger().WriteStreaming(
        "SBGS", "TimeUS,TS,GWk,GMS,ES,Mode,Nav,Fix,GS,Sec,SV,HA,VA,SA,DA,Flg",
        "QIHIIBBBIIHfffIB",
        AP_HAL::micros64(),
        cached.sbg.ekfNav.timeStamp,
        cached.sensors.gps_data.gps_week,
        cached.sensors.gps_data.ms_tow,
        cached.sbg.ekfNav.status,
        uint8_t(cached.sbg.ekfNav.status & SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK),
        uint8_t(nav_state), uint8_t(fix_type), cached.sbg.gnssPos.status, security, satellites,
        horizontal_accuracy, vertical_accuracy, speed_accuracy, differential_age_ms,
        flags);
#else
    (void)nav_state;
    (void)fix_type;
    (void)horizontal_accuracy;
    (void)vertical_accuracy;
    (void)speed_accuracy;
    (void)differential_age_ms;
#endif
}

bool AP_ExternalAHRS_SBG::healthy(void) const
{
    const uint32_t now_ms = AP_HAL::millis();
    if (option_is_set(AP_ExternalAHRS::OPTIONS::SBG_EKF_AS_GNSS)) {
        const MessageFreshness freshness = message_freshness(received, now_ms);
        return freshness.ekf_nav && freshness.status;
    }
    return last_received_ms > 0 && now_ms - last_received_ms < 500;
}

void AP_ExternalAHRS_SBG::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    memset(&status, 0, sizeof(status));

    if (cached.sensors.ins_ms != 0 && cached.sensors.gps_ms != 0) {
        status.flags.initalized = true;
    }
    if (!healthy()) {
        return;
    }

    if (state.have_location) {
        status.flags.vert_pos = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = bool(cached.sbg.ekfNav.status & SBG_ECOM_SOL_GPS1_POS_USED);
    }

    if (state.have_quaternion) {
        status.flags.attitude = true;
    }

    if (state.have_velocity) {
        status.flags.vert_vel = true;
        status.flags.horiz_vel = true;
    }
}

bool AP_ExternalAHRS_SBG::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG setup failed");
        return false;
    }
    if (option_is_set(AP_ExternalAHRS::OPTIONS::SBG_EKF_AS_GNSS)) {
        const uint32_t now_ms = AP_HAL::millis();
        if (received.ekf_nav_ms == 0) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SBG missing EKF NAV");
            return false;
        }
        if (now_ms - received.ekf_nav_ms > EKF_NAV_TIMEOUT_MS) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SBG EKF NAV stale");
            return false;
        }
        if (received.status_ms == 0) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SBG missing STATUS");
            return false;
        }
        if (now_ms - received.status_ms > STATUS_TIMEOUT_MS) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SBG STATUS stale");
            return false;
        }
    } else if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG link stale");
        return false;
    }
    return true;
}

#if AP_COMPASS_ENABLED
bool AP_ExternalAHRS_SBG::send_MagData(AP_HAL::UARTDriver *_uart)
{
    SbgEComLogMag mag_data_log {};
    mag_data_log.timeStamp = AP_HAL::micros();

    const auto &compass = AP::compass();
    if (compass.available() || compass.healthy()) {
        // TODO: consider also checking compass.last_update_usec() to only send when we have new data

        const Vector3f mag_field = compass.get_field() * 0.001f;
        mag_data_log.magnetometers[0] = mag_field[0];
        mag_data_log.magnetometers[1] = mag_field[1];
        mag_data_log.magnetometers[2] = mag_field[2];

        mag_data_log.status |= (SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAG_Y_BIT | SBG_ECOM_MAG_MAG_Z_BIT | SBG_ECOM_MAG_MAGS_IN_RANGE | SBG_ECOM_MAG_CALIBRATION_OK);
    }

    const auto &ins = AP::ins();
    if (ins.get_accel_health()) {
        const Vector3f &accel = ins.get_accel();
        mag_data_log.accelerometers[0] = accel.x;
        mag_data_log.accelerometers[1] = accel.y;
        mag_data_log.accelerometers[2] = accel.z;

        mag_data_log.status |= (SBG_ECOM_MAG_ACCEL_X_BIT | SBG_ECOM_MAG_ACCEL_Y_BIT | SBG_ECOM_MAG_ACCEL_Z_BIT | SBG_ECOM_MAG_ACCELS_IN_RANGE);
    }

    const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, (uint8_t*)&mag_data_log, sizeof(mag_data_log));
    return send_sbgMessage(_uart, msg);
}
#endif // AP_COMPASS_ENABLED

bool AP_ExternalAHRS_SBG::send_AirData(AP_HAL::UARTDriver *_uart)
{
    SbgEComLogAirData air_data_log {};
    air_data_log.timeStamp = 0;
    air_data_log.status |= SBG_ECOM_AIR_DATA_TIME_IS_DELAY;

#if AP_BARO_ENABLED
    const auto &baro = AP::baro();
    if (baro.healthy()) {
        air_data_log.pressureAbs = baro.get_pressure();
        air_data_log.altitude = baro.get_altitude();
        air_data_log.airTemperature = baro.get_temperature();

        air_data_log.status |= (SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID | SBG_ECOM_AIR_DATA_ALTITUDE_VALID | SBG_ECOM_AIR_DATA_TEMPERATURE_VALID);
    }
#endif // AP_BARO_ENABLED

#if AP_AIRSPEED_ENABLED
    auto *airspeed = AP::airspeed();
    if (airspeed != nullptr && airspeed->healthy()) {
        float airTemperature;
        if (airspeed->get_temperature(airTemperature)) {
            air_data_log.airTemperature = airTemperature;
            air_data_log.status |= SBG_ECOM_AIR_DATA_TEMPERATURE_VALID;
        }

        air_data_log.pressureDiff = airspeed->get_differential_pressure();
        air_data_log.trueAirspeed = airspeed->get_airspeed();
        air_data_log.status |= (SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID | SBG_ECOM_AIR_DATA_AIRPSEED_VALID);
    }
#endif // AP_AIRSPEED_ENABLED

    const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA, (uint8_t*)&air_data_log, sizeof(air_data_log));
    return send_sbgMessage(_uart, msg);
}

// get variances
bool AP_ExternalAHRS_SBG::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = cached.sensors.gps_data.horizontal_vel_accuracy * vel_gate_scale;
    posVar = cached.sensors.gps_data.horizontal_pos_accuracy * pos_gate_scale;
    hgtVar = cached.sensors.gps_data.vertical_pos_accuracy * hgt_gate_scale;
    magVar.zero(); // Not provided, set to 0.
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED
