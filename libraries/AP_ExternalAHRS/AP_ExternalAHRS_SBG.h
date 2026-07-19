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
  support for serial connected SBG INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_SBG_structs.h"

class AP_ExternalAHRS_SBG : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_SBG(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override { return (uart == nullptr) ? -1 : port_num; }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override { return setup_complete; };
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    void get_filter_status(nav_filter_status &status) const override;

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override { }

    // Get model/type name
    const char* get_name() const override { return "SBG"; }

    static constexpr uint8_t SBG_PACKET_SYNC1 = 0xFF;
    static constexpr uint8_t SBG_PACKET_SYNC2 = 0x5A;
    static constexpr uint8_t SBG_PACKET_ETX = 0x33;
    static constexpr uint16_t SBG_PACKET_PAYLOAD_SIZE_MAX = 100;
    static constexpr uint16_t SBG_PACKET_OVERHEAD = 9;

    struct PACKED sbgMessage {
        uint8_t msgid = 0;
        uint8_t msgclass = 0;
        uint16_t len = 0;
        uint8_t data[SBG_PACKET_PAYLOAD_SIZE_MAX] {};

        sbgMessage() = default;
        sbgMessage(uint8_t msgClass_, uint8_t msgId_);
        sbgMessage(uint8_t msgClass_, uint8_t msgId_, const uint8_t* payload, uint16_t payload_len);
    };

    enum class SBG_PACKET_PARSE_STATE : uint8_t {
        SYNC1,
        SYNC2,
        MSG,
        CLASS,
        LEN1,
        LEN2,
        DATA,
        CRC1,
        CRC2,
        ETX,
        DROP_THIS_PACKET
    };

    struct SBG_PACKET_INBOUND_STATE {
        SBG_PACKET_PARSE_STATE parser = SBG_PACKET_PARSE_STATE::SYNC1;
        uint16_t data_count = 0;
        uint16_t crc = 0;
        bool crc_valid = false;
        sbgMessage msg;
        uint32_t data_count_skip = 0;
    };

    enum class NavigationState : uint8_t {
        NO_NAVIGATION,
        INVALID_NAVIGATION,
        INERTIAL,
        GNSS_AIDED,
        ADMIN_DISABLED,
        STALE
    };

    struct TimeAnchor {
        uint16_t week = 0;
        uint32_t tow_ms = 0;
        uint32_t sbg_timestamp = 0;
        bool valid = false;
    };

    struct ReceiveTimes {
        uint32_t ekf_nav_ms = 0;
        uint32_t gps_pos_ms = 0;
        uint32_t gps_vel_ms = 0;
        uint32_t utc_ms = 0;
        uint32_t status_ms = 0;
    };

    struct MessageFreshness {
        bool ekf_nav;
        bool gps_pos;
        bool gps_vel;
        bool utc;
        bool status;
    };

    // Protocol and policy helpers are public to permit deterministic unit tests.
    static bool parse_byte(uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &inbound_state);
    static bool payload_length_valid(uint8_t msgclass, uint8_t msgid, uint16_t payload_len);
    static bool make_gps_week(const SbgEComLogUtc &utc_data, uint16_t &gps_week);
    static bool extrapolate_gps_time(const TimeAnchor &anchor, uint32_t sbg_timestamp, uint16_t &gps_week, uint32_t &tow_ms);
    static bool utc_anchor_valid(const SbgEComLogUtc &utc);
    static MessageFreshness message_freshness(const ReceiveTimes &times, uint32_t now_ms);
    static NavigationState navigation_state(const SbgEComLogEkfNav &ekf_nav, bool nav_fresh, bool disabled,
                                            float &horizontal_accuracy, float &speed_accuracy);
    static AP_GPS_FixType navigation_fix(NavigationState nav_state, const SbgEComLogEkfNav &ekf_nav,
                                         const SbgEComLogGnssPos &gnss_pos, bool gnss_pos_fresh, bool gnss_vel_fresh,
                                         float horizontal_accuracy);
    static AP_GPS_FixType gps_position_type_to_fix(uint32_t gps_pos_status);
    static AP_GPS_FixType accuracy_to_fix(float horizontal_accuracy);

protected:

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    static constexpr uint32_t EKF_NAV_TIMEOUT_MS = 200;
    static constexpr uint32_t STATUS_TIMEOUT_MS = 500;
    static constexpr uint32_t GNSS_TIMEOUT_MS = 1000;
    static constexpr uint32_t UTC_TIMEOUT_MS = 2500;
    static constexpr uint32_t TRANSITION_MESSAGE_INTERVAL_MS = 2000;

    struct Cached {
        struct {
            AP_ExternalAHRS::gps_data_message_t gps_data;
            AP_ExternalAHRS::mag_data_message_t mag_data;
            AP_ExternalAHRS::baro_data_message_t baro_data;
            AP_ExternalAHRS::ins_data_message_t ins_data;
            AP_ExternalAHRS::airspeed_data_message_t airspeed_data;

            float baro_height;
            
            uint32_t gps_ms;
            uint32_t mag_ms;
            uint32_t baro_ms;
            uint32_t ins_ms;
            uint32_t airspeed_ms;
        } sensors;
    
        struct {
            SbgEComLogUtc utc;
            SbgEComLogGnssVel gnssVel;
            SbgEComLogGnssPos gnssPos;
            SbgEComLogStatus status;
            SbgEComLogImuLegacy imuLegacy;
            SbgEComLogImuFastLegacy imuFastLegacy;
            SbgEComLogImuShort imuShort;
            SbgEComLogEkfEuler ekfEuler;
            SbgEComLogEkfQuat ekfQuat;
            SbgEComLogEkfNav ekfNav;      // biggest msg we care about, 72 bytes
            SbgEComLogAirData airData;
            SbgEComLogMag mag;
            SbgEComDeviceInfo deviceInfo;
       } sbg;
    } cached {};

    ReceiveTimes received;

    SBG_PACKET_INBOUND_STATE _inbound_state;
    TimeAnchor time_anchor;

    void handle_msg(const sbgMessage &msg);
    void update_thread();
    bool check_uart();
    static bool send_sbgMessage(AP_HAL::UARTDriver *_uart, const sbgMessage &msg);
    static void safe_copy_msg_to_object(uint8_t* dest, const uint16_t dest_len, const uint8_t* src, const uint16_t src_len);
    void publish_ekf_gps(uint32_t now_ms);
    void update_transition_messages(uint32_t now_ms, NavigationState nav_state, AP_GPS_FixType fix_type,
                                    bool nav_stale, bool status_stale, bool utc_valid);
    void write_sbg_log(NavigationState nav_state, AP_GPS_FixType fix_type, float horizontal_accuracy,
                       float vertical_accuracy, float speed_accuracy, uint32_t differential_age_ms) const;
    static void initialise_gnss_pos(SbgEComLogGnssPos &gnss_pos);
    static void initialise_utc(SbgEComLogUtc &utc);

    uint32_t send_MagData_ms = 0;
    uint32_t send_AirData_ms = 0;
    uint32_t send_mag_error_last_ms = 0;
    uint32_t send_air_error_last_ms = 0;
    static bool send_MagData(AP_HAL::UARTDriver *_uart);
    static bool send_AirData(AP_HAL::UARTDriver *_uart);

    AP_HAL::UARTDriver *uart = nullptr;
    int8_t port_num = -1;
    uint32_t baudrate = 0;
    bool setup_complete = false;
    uint32_t version_check_ms = 0;
    uint32_t last_received_ms = 0;
    uint32_t last_gps_publish_ms = 0;
    bool gps_has_published = false;

    uint8_t previous_ekf_mode = UINT8_MAX;
    NavigationState previous_nav_state = NavigationState::NO_NAVIGATION;
    bool have_previous_nav_state = false;
    bool previous_zupt = false;
    bool previous_utc_valid = false;
    bool previous_nav_stale = true;
    bool previous_status_stale = true;
    uint8_t previous_ifm = UINT8_MAX;
    uint8_t previous_spoofing = UINT8_MAX;
    uint8_t previous_osnma = UINT8_MAX;
    uint32_t mode_message_ms = 0;
    uint32_t nav_message_ms = 0;
    uint32_t zupt_message_ms = 0;
    uint32_t interference_message_ms = 0;
    uint32_t spoofing_message_ms = 0;
    uint32_t utc_message_ms = 0;
    uint32_t stale_message_ms = 0;
};

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED
