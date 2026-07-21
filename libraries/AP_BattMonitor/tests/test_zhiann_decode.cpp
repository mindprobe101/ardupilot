/*
  replay tests for the Zhiann BMS decode header. Every fixture frame is a
  real capture from live 24S packs (2026-07 bench sessions) with values
  cross-validated on the bench: against a calibrated power module for
  current, against the pack fleet's known SOC/voltages otherwise.
 */
#include <AP_gtest.h>

#include <stdlib.h>

#include <AP_BattMonitor/AP_BattMonitor_ZhiannBMS_decode.h>

using namespace ZhiannBMS;

static void hex2bytes(const char *hex, uint8_t *out)
{
    for (uint8_t i = 0; hex[2 * i] && hex[2 * i + 1]; i++) {
        char b[3] = { hex[2 * i], hex[2 * i + 1], 0 };
        out[i] = (uint8_t)strtoul(b, nullptr, 16);
    }
}

TEST(ZhiannDecode, classify_block_frames)
{
    Classified c;
    // node 0 pack frames (single-pack capture 2026-07-18)
    EXPECT_TRUE(classify(0x2E0941, c)); EXPECT_EQ(c.node, 0); EXPECT_EQ(c.type, FRAME_CELL_19_22);
    EXPECT_TRUE(classify(0x2E0951, c)); EXPECT_EQ(c.node, 0); EXPECT_EQ(c.type, FRAME_PACK_VOLT);
    // 4-pack capture: node blocks step by 0x20
    EXPECT_TRUE(classify(0x2E0961, c)); EXPECT_EQ(c.node, 1); EXPECT_EQ(c.type, FRAME_CELL_19_22);
    EXPECT_TRUE(classify(0x2E09B1, c)); EXPECT_EQ(c.node, 3); EXPECT_EQ(c.type, FRAME_PACK_VOLT);
    // 5-pack capture: node 4 appeared at 0x2E09C1 block / 0x401A104
    EXPECT_TRUE(classify(0x401A104, c)); EXPECT_EQ(c.node, 4); EXPECT_EQ(c.type, FRAME_SOC_COARSE);
}

TEST(ZhiannDecode, classify_rejects_unknown)
{
    Classified c;
    // unobserved block frame types must not classify (would otherwise
    // let alien devices bind instances - review finding)
    EXPECT_FALSE(classify(0x2E0948, c));   // type 0x07 unobserved
    EXPECT_FALSE(classify(0x2E0958, c));   // type 0x17 unobserved
    // beyond node 15
    EXPECT_FALSE(classify(0x2E0D41, c));
    // the 2s status frame and the spec heartbeat are not ours to consume
    EXPECT_FALSE(classify(0x402A100, c));
    EXPECT_FALSE(classify(0x1843FFF0, c));
}

TEST(ZhiannDecode, classify_alarm)
{
    Classified c;
    EXPECT_TRUE(classify(0x1824F002, c));
    EXPECT_EQ(c.type, FRAME_ALARM);
    EXPECT_EQ(c.node, 2);
}

TEST(ZhiannDecode, pack_volt_idle)
{
    // real frame, 100% pack at idle: V=102.72, I=0
    uint8_t d[8]; hex2bytes("1611202800000000", d);
    EXPECT_NEAR(pack_voltage(d), 102.72f, 0.001f);
    EXPECT_FLOAT_EQ(current_amps(d), 0.0f);
}

TEST(ZhiannDecode, pack_volt_under_load)
{
    // motor load test: raw s32 -23128 (discharge) at 100.31V.
    // calibrated against the power module: 2 mA/LSB -> +46.256 A
    uint8_t d[8]; hex2bytes("00002F27A8A5FFFF", d);
    EXPECT_NEAR(pack_voltage(d), 100.31f, 0.001f);
    EXPECT_NEAR(current_amps(d), 46.256f, 0.001f);
}

TEST(ZhiannDecode, temp_soc_frame)
{
    // real frame: 26.0C / 28.0C, SOC 100.0%
    uint8_t d[8]; hex2bytes("04011801E8030000", d);
    EXPECT_NEAR(temp1_c(d), 26.0f, 0.001f);
    EXPECT_NEAR(temp2_c(d), 28.0f, 0.001f);
    EXPECT_EQ(soc_fine_pct(d), 100);
}

TEST(ZhiannDecode, temp_negative)
{
    // synthetic: -5.0C on sensor 1 (0xFFCE as s16) - the unsigned-decode
    // bug from the review would read this as +6548.6C
    uint8_t d[8]; hex2bytes("CEFF1801E8030000", d);
    EXPECT_NEAR(temp1_c(d), -5.0f, 0.001f);
    EXPECT_NEAR(temp2_c(d), 28.0f, 0.001f);
}

TEST(ZhiannDecode, soc_coarse_frame)
{
    // real frame from node 2 (44% pack at 88.4V): vmir/320 = volts
    uint8_t d[8]; hex2bytes("2C00806E11000000", d);
    EXPECT_EQ(soc_coarse_pct(d), 44);
    EXPECT_NEAR(soc_voltage_mirror(d) / 320.0f, 88.4f, 0.05f);
}

TEST(ZhiannDecode, cell_frames)
{
    // real frames: cells 1-2 from the +0x02 frame (also carries count=24)
    uint8_t d[8]; hex2bytes("0000180066106610", d);
    EXPECT_EQ(d[2], 24);                    // cell count byte
    EXPECT_EQ(u16(&d[4]), 4198);            // cell 1 mV
    EXPECT_EQ(u16(&d[6]), 4198);            // cell 2 mV

    // cells 3-6
    uint8_t e[8]; hex2bytes("6710651066106410", e);
    const uint16_t expect[4] = { 4199, 4197, 4198, 4196 };
    for (uint8_t i = 0; i < 4; i++) {
        EXPECT_EQ(u16(&e[2 * i]), expect[i]);
    }
}

TEST(ZhiannDecode, cell_map_covers_24)
{
    // the map must tile cells 0..23 exactly once
    bool seen[24] = {};
    for (const auto &m : CELL_MAP) {
        for (uint8_t i = 0; i < m.ncells; i++) {
            ASSERT_LT(m.first_cell + i, 24);
            EXPECT_FALSE(seen[m.first_cell + i]);
            seen[m.first_cell + i] = true;
        }
    }
    for (uint8_t i = 0; i < 24; i++) {
        EXPECT_TRUE(seen[i]);
    }
}

TEST(ZhiannDecode, dup_detector)
{
    DupDetector det;
    uint32_t t = 1000;
    // healthy single pack: ~500ms spacing, never trips
    for (int i = 0; i < 40; i++) {
        det.feed(t); t += 500;
        EXPECT_FALSE(det.active());
    }
    // two packs collide (real timing from the node-2 collision captures:
    // alternating ~200/300ms gaps): trips within a few seconds
    for (int i = 0; i < 12; i++) {
        det.feed(t); t += (i % 2) ? 200 : 300;
    }
    EXPECT_TRUE(det.active());
    // collision resolved: recovers
    for (int i = 0; i < 40; i++) {
        det.feed(t); t += 500;
    }
    EXPECT_FALSE(det.active());
}

AP_GTEST_MAIN()
