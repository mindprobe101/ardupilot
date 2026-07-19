#include <AP_gtest.h>

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include <AP_ExternalAHRS/AP_ExternalAHRS_SBG.h>
#include <AP_Math/crc.h>

#include <vector>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using SBG = AP_ExternalAHRS_SBG;

static std::vector<uint8_t> make_frame(const uint8_t msg_class, const uint8_t msg_id,
                                       const std::vector<uint8_t> &payload)
{
    SBG::sbgMessage msg(msg_class, msg_id, payload.data(), payload.size());
    const uint16_t crc = crc16_ccitt_r(reinterpret_cast<const uint8_t *>(&msg), msg.len + 4, 0, 0);
    std::vector<uint8_t> frame {
        SBG::SBG_PACKET_SYNC1,
        SBG::SBG_PACKET_SYNC2,
        msg_id,
        msg_class,
        uint8_t(msg.len),
        uint8_t(msg.len >> 8),
    };
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(uint8_t(crc));
    frame.push_back(uint8_t(crc >> 8));
    frame.push_back(SBG::SBG_PACKET_ETX);
    return frame;
}

static uint8_t feed_parser(const std::vector<uint8_t> &bytes, SBG::SBG_PACKET_INBOUND_STATE &state)
{
    uint8_t packet_count = 0;
    for (const uint8_t byte : bytes) {
        if (SBG::parse_byte(byte, state.msg, state)) {
            packet_count++;
        }
    }
    return packet_count;
}

static SbgEComLogEkfNav valid_ekf_nav()
{
    SbgEComLogEkfNav nav {};
    nav.status = SBG_ECOM_SOL_MODE_NAV_POSITION |
                 SBG_ECOM_SOL_POSITION_VALID |
                 SBG_ECOM_SOL_VELOCITY_VALID |
                 SBG_ECOM_SOL_GPS1_POS_USED;
    nav.position[0] = 12.5;
    nav.position[1] = 77.5;
    nav.position[2] = 100.0;
    nav.positionStdDev[0] = 0.06f;
    nav.positionStdDev[1] = 0.08f;
    nav.positionStdDev[2] = 0.2f;
    nav.velocity[0] = 1.0f;
    nav.velocity[1] = 2.0f;
    nav.velocity[2] = 3.0f;
    nav.velocityStdDev[0] = 0.1f;
    nav.velocityStdDev[1] = 0.2f;
    nav.velocityStdDev[2] = 0.2f;
    return nav;
}

static SbgEComLogGnssPos clean_gnss_pos(const SbgEComGpsPosType type)
{
    SbgEComLogGnssPos pos {};
    pos.status = uint32_t(type) << SBG_ECOM_GPS_POS_TYPE_SHIFT;
    pos.statusExt = uint32_t(SBG_ECOM_GNSS_IFM_STATUS_CLEAN) |
                    (uint32_t(SBG_ECOM_GNSS_SPOOFING_STATUS_CLEAN) << SBG_ECOM_GPS_POS_SPOOFING_SHIFT) |
                    (uint32_t(SBG_ECOM_GNSS_OSNMA_STATUS_VALID) << SBG_ECOM_GPS_POS_OSNMA_SHIFT);
    return pos;
}

TEST(SBGParser, ValidCrcAndEtx)
{
    SBG::SBG_PACKET_INBOUND_STATE state;
    const std::vector<uint8_t> payload { 1, 2, 3, 4, 5 };
    const auto frame = make_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, payload);

    EXPECT_EQ(feed_parser(frame, state), 1);
    EXPECT_EQ(state.msg.msgclass, SBG_ECOM_CLASS_LOG_ECOM_0);
    EXPECT_EQ(state.msg.msgid, SBG_ECOM_LOG_STATUS);
    EXPECT_EQ(state.msg.len, payload.size());
    EXPECT_EQ(memcmp(state.msg.data, payload.data(), payload.size()), 0);
}

TEST(SBGParser, InvalidCrcAndEtxResynchronise)
{
    const auto good_frame = make_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS,
                                       std::vector<uint8_t> { 9, 8, 7 });

    auto bad_crc = good_frame;
    bad_crc[bad_crc.size() - 3] ^= 0x80;
    bad_crc.insert(bad_crc.end(), good_frame.begin(), good_frame.end());
    SBG::SBG_PACKET_INBOUND_STATE crc_state;
    EXPECT_EQ(feed_parser(bad_crc, crc_state), 1);

    auto bad_etx = good_frame;
    bad_etx.back() = 0;
    bad_etx.insert(bad_etx.end(), good_frame.begin(), good_frame.end());
    SBG::SBG_PACKET_INBOUND_STATE etx_state;
    EXPECT_EQ(feed_parser(bad_etx, etx_state), 1);

    auto truncated = good_frame;
    truncated.pop_back();
    truncated.insert(truncated.end(), good_frame.begin(), good_frame.end());
    SBG::SBG_PACKET_INBOUND_STATE truncated_state;
    EXPECT_EQ(feed_parser(truncated, truncated_state), 1);
}

TEST(SBGParser, OversizedPacketSkipsExactlyOneFrame)
{
    constexpr uint16_t oversized_length = SBG::SBG_PACKET_PAYLOAD_SIZE_MAX + 1;
    std::vector<uint8_t> stream {
        SBG::SBG_PACKET_SYNC1,
        SBG::SBG_PACKET_SYNC2,
        SBG_ECOM_LOG_STATUS,
        SBG_ECOM_CLASS_LOG_ECOM_0,
        uint8_t(oversized_length),
        uint8_t(oversized_length >> 8),
    };
    stream.insert(stream.end(), oversized_length, SBG::SBG_PACKET_SYNC1);
    stream.push_back(0);
    stream.push_back(0);
    stream.push_back(SBG::SBG_PACKET_ETX);

    const auto good_frame = make_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME,
                                       std::vector<uint8_t> { 1, 2, 3 });
    stream.insert(stream.end(), good_frame.begin(), good_frame.end());

    SBG::SBG_PACKET_INBOUND_STATE state;
    EXPECT_EQ(feed_parser(stream, state), 1);
    EXPECT_EQ(state.msg.msgid, SBG_ECOM_LOG_UTC_TIME);
}

TEST(SBGPayload, MinimumAndExtensibleLengths)
{
    EXPECT_FALSE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, 51));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, 52));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, 67));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, 80));

    EXPECT_FALSE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, 20));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, 21));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, 33));

    EXPECT_FALSE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, 21));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, 22));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, 27));

    EXPECT_FALSE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, 71));
    EXPECT_TRUE(SBG::payload_length_valid(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, 72));
    EXPECT_FALSE(SBG::payload_length_valid(99, SBG_ECOM_LOG_STATUS, 100));
}

TEST(SBGTime, WeekAndTimestampRollover)
{
    SbgEComLogUtc utc {};
    utc.year = 2026;
    utc.month = 7;
    utc.day = 20;
    utc.hour = 12;
    utc.minute = 34;
    utc.second = 56;
    utc.gpsTimeOfWeek = 131714000;

    uint16_t week = 0;
    ASSERT_TRUE(SBG::make_gps_week(utc, week));
    EXPECT_EQ(week, 2428);

    SBG::TimeAnchor anchor;
    anchor.week = 2428;
    anchor.tow_ms = AP_MSEC_PER_WEEK - 100;
    anchor.sbg_timestamp = 0xFFFFFF00U;
    anchor.valid = true;
    uint32_t tow_ms = 0;
    ASSERT_TRUE(SBG::extrapolate_gps_time(anchor, anchor.sbg_timestamp + 200000U, week, tow_ms));
    EXPECT_EQ(week, 2429);
    EXPECT_EQ(tow_ms, 100U);

    anchor.week = 2429;
    anchor.tow_ms = 0;
    anchor.sbg_timestamp = 1000;
    ASSERT_TRUE(SBG::extrapolate_gps_time(anchor, 0, week, tow_ms));
    EXPECT_EQ(week, 2428);
    EXPECT_EQ(tow_ms, AP_MSEC_PER_WEEK - 1);
}

TEST(SBGTime, UtcValidityAndInvalidCalendarFields)
{
    SbgEComLogUtc utc {};
    utc.year = 2026;
    utc.month = 7;
    utc.day = 20;
    utc.gpsTimeOfWeek = 86418000;
    utc.status = (uint16_t(SBG_ECOM_CLOCK_STEERING) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
                 (uint16_t(SBG_ECOM_UTC_VALID) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT);
    EXPECT_TRUE(SBG::utc_anchor_valid(utc));

    utc.status = (uint16_t(SBG_ECOM_CLOCK_FREE_RUNNING) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
                 (uint16_t(SBG_ECOM_UTC_VALID) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT);
    EXPECT_TRUE(SBG::utc_anchor_valid(utc));

    utc.status = (uint16_t(SBG_ECOM_CLOCK_VALID) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
                 (uint16_t(SBG_ECOM_UTC_INVALID) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT);
    EXPECT_FALSE(SBG::utc_anchor_valid(utc));

    utc.status = (uint16_t(SBG_ECOM_CLOCK_ERROR) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
                 (uint16_t(SBG_ECOM_UTC_VALID) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT);
    EXPECT_FALSE(SBG::utc_anchor_valid(utc));

    uint16_t week;
    utc.day = 32;
    EXPECT_FALSE(SBG::make_gps_week(utc, week));
    utc.day = 20;
    utc.nanoSecond = INT32_MAX;
    EXPECT_FALSE(SBG::make_gps_week(utc, week));
}

TEST(SBGTime, MessageFreshnessExpiresIndependently)
{
    constexpr uint32_t now_ms = 10000;
    SBG::ReceiveTimes times;
    times.ekf_nav_ms = now_ms;
    times.gps_pos_ms = now_ms;
    times.gps_vel_ms = now_ms;
    times.utc_ms = now_ms;
    times.status_ms = now_ms;

    auto freshness = SBG::message_freshness(times, now_ms);
    EXPECT_TRUE(freshness.ekf_nav);
    EXPECT_TRUE(freshness.gps_pos);
    EXPECT_TRUE(freshness.gps_vel);
    EXPECT_TRUE(freshness.utc);
    EXPECT_TRUE(freshness.status);

    times.ekf_nav_ms = now_ms - 201;
    freshness = SBG::message_freshness(times, now_ms);
    EXPECT_FALSE(freshness.ekf_nav);
    EXPECT_TRUE(freshness.gps_pos);
    times.ekf_nav_ms = now_ms;

    times.gps_pos_ms = now_ms - 1001;
    freshness = SBG::message_freshness(times, now_ms);
    EXPECT_FALSE(freshness.gps_pos);
    EXPECT_TRUE(freshness.gps_vel);
    times.gps_pos_ms = now_ms;

    times.gps_vel_ms = now_ms - 1001;
    freshness = SBG::message_freshness(times, now_ms);
    EXPECT_TRUE(freshness.gps_pos);
    EXPECT_FALSE(freshness.gps_vel);
    times.gps_vel_ms = now_ms;

    times.utc_ms = now_ms - 2501;
    freshness = SBG::message_freshness(times, now_ms);
    EXPECT_FALSE(freshness.utc);
    EXPECT_TRUE(freshness.status);
    times.utc_ms = now_ms;

    times.status_ms = now_ms - 501;
    freshness = SBG::message_freshness(times, now_ms);
    EXPECT_TRUE(freshness.utc);
    EXPECT_FALSE(freshness.status);

    times.ekf_nav_ms = UINT32_MAX - 49;
    freshness = SBG::message_freshness(times, 50);
    EXPECT_TRUE(freshness.ekf_nav);
}

TEST(SBGNavigation, StateValidationAndModifiers)
{
    auto nav = valid_ekf_nav();
    float horizontal_accuracy;
    float speed_accuracy;

    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::GNSS_AIDED);
    EXPECT_NEAR(horizontal_accuracy, 0.1f, 1.0e-6f);
    EXPECT_NEAR(speed_accuracy, 0.3f, 1.0e-6f);

    nav.status &= ~SBG_ECOM_SOL_GPS1_POS_USED;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INERTIAL);
    nav.status |= SBG_ECOM_SOL_ZUPT_USED;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INERTIAL);

    EXPECT_EQ(SBG::navigation_state(nav, true, true, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::ADMIN_DISABLED);
    EXPECT_EQ(SBG::navigation_state(nav, false, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::STALE);

    nav = valid_ekf_nav();
    nav.status &= ~SBG_ECOM_SOL_POSITION_VALID;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INVALID_NAVIGATION);

    nav = valid_ekf_nav();
    nav.position[0] = NAN;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INVALID_NAVIGATION);
    nav = valid_ekf_nav();
    nav.velocity[1] = INFINITY;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INVALID_NAVIGATION);
    nav = valid_ekf_nav();
    nav.positionStdDev[0] = -1.0f;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INVALID_NAVIGATION);
    nav = valid_ekf_nav();
    nav.positionStdDev[0] = 100.0f;
    EXPECT_EQ(SBG::navigation_state(nav, true, false, horizontal_accuracy, speed_accuracy),
              SBG::NavigationState::INVALID_NAVIGATION);
}

TEST(SBGNavigation, AccuracyCeilings)
{
    EXPECT_EQ(SBG::accuracy_to_fix(0.09f), AP_GPS_FixType::RTK_FIXED);
    EXPECT_EQ(SBG::accuracy_to_fix(0.10f), AP_GPS_FixType::RTK_FLOAT);
    EXPECT_EQ(SBG::accuracy_to_fix(0.30f), AP_GPS_FixType::DGPS);
    EXPECT_EQ(SBG::accuracy_to_fix(1.20f), AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(SBG::accuracy_to_fix(100.0f), AP_GPS_FixType::NONE);
    EXPECT_EQ(SBG::accuracy_to_fix(NAN), AP_GPS_FixType::NONE);
    EXPECT_EQ(SBG::accuracy_to_fix(INFINITY), AP_GPS_FixType::NONE);
    EXPECT_EQ(SBG::accuracy_to_fix(-1.0f), AP_GPS_FixType::NONE);
}

TEST(SBGNavigation, EveryRawGnssType)
{
    const struct {
        SbgEComGpsPosType type;
        AP_GPS_FixType expected;
    } cases[] {
        { SBG_ECOM_POS_NO_SOLUTION, AP_GPS_FixType::FIX_3D },
        { SBG_ECOM_POS_UNKNOWN_TYPE, AP_GPS_FixType::FIX_3D },
        { SBG_ECOM_POS_SINGLE, AP_GPS_FixType::FIX_3D },
        { SBG_ECOM_POS_PSRDIFF, AP_GPS_FixType::DGPS },
        { SBG_ECOM_POS_SBAS, AP_GPS_FixType::DGPS },
        { SBG_ECOM_POS_OMNISTAR, AP_GPS_FixType::RTK_FLOAT },
        { SBG_ECOM_POS_RTK_FLOAT, AP_GPS_FixType::RTK_FLOAT },
        { SBG_ECOM_POS_RTK_INT, AP_GPS_FixType::RTK_FIXED },
        { SBG_ECOM_POS_PPP_FLOAT, AP_GPS_FixType::RTK_FLOAT },
        { SBG_ECOM_POS_PPP_INT, AP_GPS_FixType::RTK_FIXED },
        { SBG_ECOM_POS_FIXED, AP_GPS_FixType::FIX_3D },
    };
    const auto nav = valid_ekf_nav();

    for (const auto &test : cases) {
        const auto pos = clean_gnss_pos(test.type);
        EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
                  test.expected) << "GNSS type " << unsigned(test.type);
    }
}

TEST(SBGNavigation, GnssQualityCapsAndFreshness)
{
    auto nav = valid_ekf_nav();
    auto pos = clean_gnss_pos(SBG_ECOM_POS_RTK_INT);

    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::RTK_FIXED);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.20f),
              AP_GPS_FixType::RTK_FLOAT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.50f),
              AP_GPS_FixType::DGPS);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 2.0f),
              AP_GPS_FixType::FIX_3D);

    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, false, true, 0.05f),
              AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, false, 0.05f),
              AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::INERTIAL, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::FIX_3D);

    pos.status = SBG_ECOM_GPS_POS_STATUS_INSUFFICIENT_OBS |
                 (uint32_t(SBG_ECOM_POS_RTK_INT) << SBG_ECOM_GPS_POS_TYPE_SHIFT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::FIX_3D);

    pos = clean_gnss_pos(SBG_ECOM_POS_RTK_INT);
    pos.statusExt = (pos.statusExt & ~(SBG_ECOM_GPS_POS_EXT_STATUS_MASK << SBG_ECOM_GPS_POS_IFM_SHIFT)) |
                    (uint32_t(SBG_ECOM_GNSS_IFM_STATUS_CRITICAL) << SBG_ECOM_GPS_POS_IFM_SHIFT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::FIX_3D);

    pos = clean_gnss_pos(SBG_ECOM_POS_RTK_INT);
    pos.statusExt = (pos.statusExt & ~(SBG_ECOM_GPS_POS_EXT_STATUS_MASK << SBG_ECOM_GPS_POS_SPOOFING_SHIFT)) |
                    (uint32_t(SBG_ECOM_GNSS_SPOOFING_STATUS_SINGLE) << SBG_ECOM_GPS_POS_SPOOFING_SHIFT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::FIX_3D);

    pos = clean_gnss_pos(SBG_ECOM_POS_RTK_INT);
    pos.statusExt = (pos.statusExt & ~(SBG_ECOM_GPS_POS_EXT_STATUS_MASK << SBG_ECOM_GPS_POS_OSNMA_SHIFT)) |
                    (uint32_t(SBG_ECOM_GNSS_OSNMA_STATUS_SPOOFED) << SBG_ECOM_GPS_POS_OSNMA_SHIFT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::FIX_3D);

    nav.status |= SBG_ECOM_SOL_ZUPT_USED;
    pos = clean_gnss_pos(SBG_ECOM_POS_RTK_INT);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::GNSS_AIDED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::RTK_FIXED);
    EXPECT_EQ(SBG::navigation_fix(SBG::NavigationState::ADMIN_DISABLED, nav, pos, true, true, 0.05f),
              AP_GPS_FixType::NONE);
}

#else

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#endif

AP_GTEST_MAIN()
