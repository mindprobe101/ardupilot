/*
  replay tests for the Zhiann BMS decode header. Payload fixtures are from
  live 24S packs (2026-07 bench sessions) except where an edge case is
  explicitly marked synthetic.
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
    EXPECT_TRUE(classify(0x1824F080, c));
    EXPECT_TRUE(classify(0x1824F0EF, c));

    // Reject lookalike PF 0x24 traffic with the wrong priority,
    // destination class, or source-address class.
    EXPECT_FALSE(classify(0x1024F002, c));
    EXPECT_FALSE(classify(0x18240102, c));
    EXPECT_FALSE(classify(0x1824F0F0, c));
}

TEST(ZhiannDecode, canonical_dlc)
{
    EXPECT_TRUE(valid_dlc(FRAME_PACK_VOLT, 8));
    EXPECT_TRUE(valid_dlc(FRAME_TEMP_SOC, 8));
    EXPECT_TRUE(valid_dlc(FRAME_CELL_23_24, 4));
    EXPECT_TRUE(valid_dlc(FRAME_ALARM, 8));
    EXPECT_FALSE(valid_dlc(FRAME_PACK_VOLT, 4));
    EXPECT_FALSE(valid_dlc(FRAME_CELL_23_24, 8));
    EXPECT_FALSE(valid_dlc(0x77, 8));
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

TEST(ZhiannDecode, current_extreme_is_defined)
{
    uint8_t min_raw[8]; hex2bytes("0000000000000080", min_raw);
    uint8_t max_raw[8]; hex2bytes("00000000FFFFFF7F", max_raw);
    EXPECT_GT(current_amps(min_raw), 0.0f);
    EXPECT_LT(current_amps(max_raw), 0.0f);
    EXPECT_FALSE(current_valid(current_amps(min_raw)));
    EXPECT_FALSE(current_valid(current_amps(max_raw)));
}

TEST(ZhiannDecode, physical_plausibility)
{
    EXPECT_TRUE(pack_voltage_valid(102.72f));
    EXPECT_FALSE(pack_voltage_valid(0.0f));
    EXPECT_FALSE(pack_voltage_valid(655.35f));
    EXPECT_TRUE(temperature_valid(-5.0f));
    EXPECT_FALSE(temperature_valid(3276.7f));
}

TEST(ZhiannDecode, temp_soc_frame)
{
    // real frame: 26.0C / 28.0C, SOC 100.0%
    uint8_t d[8]; hex2bytes("04011801E8030000", d);
    EXPECT_NEAR(temp1_c(d), 26.0f, 0.001f);
    EXPECT_NEAR(temp2_c(d), 28.0f, 0.001f);
    EXPECT_TRUE(soc_fine_valid(d));
    EXPECT_EQ(soc_fine_tenths(d), 1000);
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
    EXPECT_TRUE(pack_vmir_coherent(88.4f, soc_voltage_mirror(d), 1.0f));
    EXPECT_FALSE(pack_vmir_coherent(100.0f, soc_voltage_mirror(d), 1.0f));
}

TEST(ZhiannDecode, soc_coarse_masks_status_bits)
{
    // Real load frame: fine SOC was 94.9%; byte 0 high bit and byte 1
    // contain dynamic status data, while low seven bits remain 94.
    uint8_t load[8]; hex2bytes("DEE9FF7D11000000", load);
    EXPECT_EQ(soc_coarse_pct(load), 94);

    // Real standby/wake variants of the same 44% pack.
    uint8_t standby[8]; hex2bytes("2CFF7F6E00000000", standby);
    uint8_t wake[8]; hex2bytes("ACFF7F6E11000000", wake);
    EXPECT_EQ(soc_coarse_pct(standby), 44);
    EXPECT_EQ(soc_coarse_pct(wake), 44);
    EXPECT_TRUE(soc_coarse_valid(standby));

    uint8_t invalid[8]; hex2bytes("7F00000000000000", invalid);
    EXPECT_FALSE(soc_coarse_valid(invalid));
}

TEST(ZhiannDecode, freshness_wrap_and_consumption_gap)
{
    EXPECT_TRUE(fresh_ms(1100, 1000, 100));
    EXPECT_FALSE(fresh_ms(1101, 1000, 100));
    EXPECT_FALSE(fresh_ms(1000, 0, 5000));
    EXPECT_TRUE(fresh_ms(25, UINT32_MAX - 25, 51));
    EXPECT_EQ(consumption_dt_us(500000), 500000U);
    EXPECT_EQ(consumption_dt_us(uint64_t(UINT32_MAX) + 1), UINT32_MAX);
}

TEST(ZhiannDecode, cell_snapshot_is_atomic_and_validates_count)
{
    CellAccumulator accumulator;
    uint8_t cell12[8]; hex2bytes("0000180066106610", cell12);
    uint8_t four_cells[8]; hex2bytes("6710651066106410", four_cells);
    uint8_t two_cells[8]; hex2bytes("6710651000000000", two_cells);
    const uint8_t types[] = {
        FRAME_CELL_1_2, FRAME_CELL_3_6, FRAME_CELL_7_10,
        FRAME_CELL_11_14, FRAME_CELL_15_18, FRAME_CELL_19_22,
    };
    uint32_t now_ms = 1000;
    for (uint8_t i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        EXPECT_FALSE(accumulator.feed(types[i], i == 0 ? cell12 : four_cells,
                                      now_ms, 250));
        now_ms += 10;
    }
    EXPECT_TRUE(accumulator.feed(FRAME_CELL_23_24, two_cells, now_ms, 250));
    EXPECT_EQ(accumulator.cell_count(), 24);
    EXPECT_EQ(accumulator.cells()[0], 4198);
    EXPECT_EQ(accumulator.cells()[23], 4197);

    // The count is a full LE u16; a corrupt high byte must not alias 24.
    cell12[3] = 0xFF;
    now_ms += 500;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_1_2, cell12, now_ms, 250));
    for (uint8_t i = 1; i < sizeof(types) / sizeof(types[0]); i++) {
        now_ms += 10;
        EXPECT_FALSE(accumulator.feed(types[i], four_cells, now_ms, 250));
    }
    now_ms += 10;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_23_24, two_cells, now_ms, 250));

    // A delayed final slice cannot complete a mixed-generation snapshot.
    cell12[3] = 0;
    now_ms += 500;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_1_2, cell12, now_ms, 250));
    for (uint8_t i = 1; i < sizeof(types) / sizeof(types[0]); i++) {
        now_ms += 10;
        EXPECT_FALSE(accumulator.feed(types[i], four_cells, now_ms, 250));
    }
    now_ms += 300;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_23_24, two_cells, now_ms, 250));

    // Canonical order is part of the collision defense: an interleaved or
    // duplicated slice invalidates the pending generation.
    now_ms += 500;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_1_2, cell12, now_ms, 250));
    now_ms += 10;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_7_10, four_cells, now_ms, 250));
    now_ms += 10;
    EXPECT_FALSE(accumulator.feed(FRAME_CELL_3_6, four_cells, now_ms, 250));
}

TEST(ZhiannDecode, consumed_capacity_from_soc)
{
    EXPECT_NEAR(consumed_mah_from_soc(44000.0f, 1000), 0.0f, 0.01f);
    EXPECT_NEAR(consumed_mah_from_soc(44000.0f, 440), 24640.0f, 0.01f);
    EXPECT_NEAR(consumed_mah_from_soc(44000.0f, 0), 44000.0f, 0.01f);
    // Initial SOC seeds capacity; later SOC may raise but never erase the
    // calibrated-current integral.
    EXPECT_NEAR(reconciled_consumed_mah(0, 44000.0f, 440, false),
                24640.0f, 0.01f);
    EXPECT_NEAR(reconciled_consumed_mah(25000.0f, 44000.0f, 500, true),
                25000.0f, 0.01f);
    EXPECT_NEAR(reconciled_consumed_mah(25000.0f, 44000.0f, 400, true),
                26400.0f, 0.01f);
}

TEST(ZhiannDecode, invalid_soc_is_rejected)
{
    uint8_t fine[8]; hex2bytes("00000000E9030000", fine); // 100.1%
    EXPECT_FALSE(soc_fine_valid(fine));
}

TEST(ZhiannDecode, cell_sum_coherence_detects_collision_mix)
{
    uint16_t cells[24];
    for (uint8_t i = 0; i < 24; i++) {
        cells[i] = 4265;
    }
    // Approximate real collision sequence: 88.89V PACK followed by cells
    // from a ~102.36V pack on the same node.
    EXPECT_FALSE(pack_cells_coherent(88.89f, cells, 24, 1.0f));
    EXPECT_TRUE(pack_cells_coherent(102.36f, cells, 24, 1.0f));
    cells[0] = UINT16_MAX;
    EXPECT_FALSE(pack_cells_coherent(102.36f, cells, 24, 1.0f));
    // An impossible individual cell must not hide behind a plausible sum.
    cells[0] = 60000;
    for (uint8_t i = 1; i < 24; i++) {
        cells[i] = 1739;
    }
    EXPECT_FALSE(pack_cells_coherent(100.0f, cells, 24, 1.0f));
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
    EXPECT_TRUE(det.qualified());
    // two packs collide (real timing from the node-2 collision captures:
    // alternating ~200/300ms gaps): trips within a few seconds
    for (int i = 0; i < 12; i++) {
        det.feed(t); t += (i % 2) ? 200 : 300;
    }
    EXPECT_TRUE(det.active());
    EXPECT_FALSE(det.qualified());
    // collision resolved: recovers
    for (int i = 0; i < 40; i++) {
        det.feed(t); t += 500;
    }
    EXPECT_FALSE(det.active());
    EXPECT_TRUE(det.qualified());

    det.reset_qualification();
    EXPECT_FALSE(det.qualified());

}

TEST(ZhiannDecode, coherence_detector_holds_until_clean_run)
{
    CoherenceDetector detector;
    EXPECT_FALSE(detector.active());
    detector.feed(false);
    EXPECT_TRUE(detector.active());
    for (uint8_t i = 0; i < 19; i++) {
        detector.feed(true);
        EXPECT_TRUE(detector.active());
    }
    detector.feed(true);
    EXPECT_FALSE(detector.active());
}

AP_GTEST_MAIN()
