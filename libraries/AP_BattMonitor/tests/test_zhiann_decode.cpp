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
    // the spec heartbeat is not ours to consume. (The 2s frame at
    // 0x402A10n IS consumed since 2026-08-14 for its per-pack identity -
    // see classify_pack_id_frame.)
    EXPECT_FALSE(classify(0x1843FFF0, c));
    // adjacent 2s-frame ids outside the node range must still be rejected
    EXPECT_FALSE(classify(0x402A110, c));
    EXPECT_FALSE(classify(0x402A0FF, c));
}

TEST(ZhiannDecode, classify_alarm)
{
    Classified c;
    EXPECT_TRUE(classify(0x1824F002, c));
    EXPECT_EQ(c.type, FRAME_ALARM);
    EXPECT_EQ(c.node, 2);
    EXPECT_TRUE(classify(0x1824F080, c));
    EXPECT_TRUE(classify(0x1824F0EF, c));

    // Any priority and the null/global destinations are accepted: alarm
    // delivery must not hinge on unverified priority or exact-destination
    // assumptions.
    EXPECT_TRUE(classify(0x1024F002, c));
    EXPECT_EQ(c.type, FRAME_ALARM);
    EXPECT_TRUE(classify(0x0024F002, c));   // priority 0
    EXPECT_TRUE(classify(0x18240002, c));   // PS 0x00 (null destination)
    EXPECT_TRUE(classify(0x1824FF02, c));   // PS 0xFF (global destination)

    // Reject lookalike traffic: mid-range destination, source address
    // above 0xEF, or a data-page/reserved bit set (a different PGN).
    EXPECT_FALSE(classify(0x18240102, c));
    EXPECT_FALSE(classify(0x1824F0F0, c));
    EXPECT_FALSE(classify(0x1924F002, c));  // DP = 1
    EXPECT_FALSE(classify(0x1A24F002, c));  // reserved bit = 1
}

TEST(ZhiannDecode, canonical_dlc)
{
    EXPECT_TRUE(valid_dlc(FRAME_PACK_VOLT, 8));
    EXPECT_TRUE(valid_dlc(FRAME_TEMP_SOC, 8));
    EXPECT_TRUE(valid_dlc(FRAME_CELL_23_24, 4));
    // both alarm words fit in bytes 0-3, so short alarm frames still count
    EXPECT_TRUE(valid_dlc(FRAME_ALARM, 8));
    EXPECT_TRUE(valid_dlc(FRAME_ALARM, 4));
    EXPECT_FALSE(valid_dlc(FRAME_ALARM, 3));
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
    // 1 mA/LSB (2026-08-14, from the pack's own 44.0Ah rated capacity and
    // coulomb counting against its SOC) -> +23.128 A
    uint8_t d[8]; hex2bytes("00002F27A8A5FFFF", d);
    EXPECT_NEAR(pack_voltage(d), 100.31f, 0.001f);
    EXPECT_NEAR(current_amps(d), 23.128f, 0.001f);
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

TEST(ZhiannDecode, invalid_soc_is_rejected)
{
    uint8_t fine[8]; hex2bytes("00000000E9030000", fine); // 100.1%
    EXPECT_FALSE(soc_fine_valid(fine));
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

TEST(ZhiannDecode, pack_volt_partial_validation)
{
    // synthetic: plausible voltage (102.72V) with a corrupt/sentinel
    // current register (INT32_MIN). Voltage and current are judged
    // independently, so the frame keeps liveness while current is dropped.
    uint8_t d[8]; hex2bytes("1611202800000080", d);
    EXPECT_NEAR(pack_voltage(d), 102.72f, 0.001f);
    EXPECT_TRUE(pack_voltage_valid(pack_voltage(d)));
    EXPECT_FALSE(current_valid(current_amps(d)));

    // an implausible voltage rejects the frame regardless of the current
    uint8_t bad_v[8]; hex2bytes("16110000E8030000", bad_v);
    EXPECT_FALSE(pack_voltage_valid(pack_voltage(bad_v)));
    EXPECT_TRUE(current_valid(current_amps(bad_v)));
}

TEST(ZhiannDecode, temp_single_sensor_selection)
{
    float out = 0;
    // both plausible: the hotter one is published
    EXPECT_TRUE(select_temperature(26.0f, 28.0f, out));
    EXPECT_NEAR(out, 28.0f, 0.001f);
    // one failed sensor (s16 limit ~3276.7C) must not blank temperature
    EXPECT_TRUE(select_temperature(3276.7f, 28.0f, out));
    EXPECT_NEAR(out, 28.0f, 0.001f);
    EXPECT_TRUE(select_temperature(26.0f, -3276.8f, out));
    EXPECT_NEAR(out, 26.0f, 0.001f);
    // both failed: no update at all
    EXPECT_FALSE(select_temperature(3276.7f, -3276.8f, out));
}

TEST(ZhiannDecode, classify_pack_id_frame)
{
    Classified c {};
    // 2s identity frame, one per node
    EXPECT_TRUE(classify(0x402A100, c));
    EXPECT_EQ(c.node, 0); EXPECT_EQ(c.type, FRAME_PACK_ID);
    EXPECT_TRUE(classify(0x402A102, c));
    EXPECT_EQ(c.node, 2); EXPECT_EQ(c.type, FRAME_PACK_ID);
    EXPECT_EQ(expected_dlc(FRAME_PACK_ID), 8);
    // real capture: node 2 identity frame of pack FFFFBDBD
    uint8_t d[8]; hex2bytes("AF040000FFFFBDBD", d);
    EXPECT_EQ(pack_id(d), 0xFFFFBDBDUL);
}

TEST(ZhiannDecode, identity_dup_single_pack_never_trips)
{
    IdentityDupDetector id;
    // a lone pack repeats one id at 0.5Hz for a long time
    for (uint32_t t = 1000; t < 120000; t += 2000) {
        id.feed(0xFFFF5B8CUL, t);
        EXPECT_FALSE(id.active(t));
    }
    EXPECT_EQ(id.latest_id(), 0xFFFF5B8CUL);
}

TEST(ZhiannDecode, identity_dup_two_packs_detected)
{
    IdentityDupDetector id;
    // real node-2 collision: two ids alternating about once a second
    uint32_t t = 1000;
    id.feed(0xFFFFBDBDUL, t);
    EXPECT_FALSE(id.active(t));         // one id alone is not a duplicate
    t += 1000;
    id.feed(0xFFFFF6DBUL, t);
    EXPECT_TRUE(id.active(t));          // second distinct id: duplicate
    EXPECT_TRUE(id.id_a() == 0xFFFFBDBDUL || id.id_b() == 0xFFFFBDBDUL);
    EXPECT_TRUE(id.id_a() == 0xFFFFF6DBUL || id.id_b() == 0xFFFFF6DBUL);
    // sustained alternation keeps it active
    for (int i = 0; i < 20; i++) {
        t += 1000;
        id.feed((i & 1) ? 0xFFFFBDBDUL : 0xFFFFF6DBUL, t);
        EXPECT_TRUE(id.active(t));
    }
    // one pack removed: the stale entry ages out of the window
    for (int i = 0; i < 10; i++) {
        t += 1000;
        id.feed(0xFFFFF6DBUL, t);
    }
    EXPECT_FALSE(id.active(t));
}

TEST(ZhiannDecode, identity_dup_ignores_sentinels_and_survives_wrap)
{
    IdentityDupDetector id;
    id.feed(0, 1000);
    id.feed(0xFFFFFFFFUL, 2000);
    EXPECT_FALSE(id.active(2000));
    EXPECT_EQ(id.latest_id(), 0u);
    // millis() wrap must not fake freshness or staleness
    IdentityDupDetector w;
    const uint32_t near_wrap = 0xFFFFF000UL;
    w.feed(0xAAAA1111UL, near_wrap);
    w.feed(0xBBBB2222UL, near_wrap + 1000);
    EXPECT_TRUE(w.active(near_wrap + 1000));
    EXPECT_TRUE(w.active(uint32_t(near_wrap + 5000)));   // wrapped
    EXPECT_FALSE(w.active(uint32_t(near_wrap + 60000))); // wrapped, stale
}

TEST(ZhiannDecode, soc_cadence_single_pack_never_trips)
{
    SocCadenceDupDetector d;
    // one pack: 201ms median measured across every clean capture
    for (uint32_t t = 1000; t < 60000; t += 201) {
        d.feed(t);
        EXPECT_FALSE(d.active(t));
    }
}

TEST(ZhiannDecode, soc_cadence_two_packs_detected)
{
    SocCadenceDupDetector d;
    uint32_t t = 1000;
    for (int i = 0; i < 6; i++) { t += 201; d.feed(t); }
    EXPECT_FALSE(d.active(t));
    // second pack joins: merged interval halves to ~100ms
    for (int i = 0; i < 10; i++) { t += 100; d.feed(t); }
    EXPECT_TRUE(d.active(t));
    // pack removed: returns to 201ms and the verdict clears
    for (int i = 0; i < 30; i++) { t += 201; d.feed(t); }
    EXPECT_FALSE(d.active(t));
}

TEST(ZhiannDecode, soc_cadence_ignores_retransmit_burst)
{
    // A pack alone on the bus retransmits every ~4ms until something ACKs,
    // and the CAN RX queue drains back to back at startup. Neither is a
    // duplicate; this is what caused the historical boot-time false alarms.
    SocCadenceDupDetector d;
    uint32_t t = 1000;
    for (int i = 0; i < 500; i++) { t += 4; d.feed(t); }
    EXPECT_FALSE(d.active(t));
    for (int i = 0; i < 200; i++) { t += 1; d.feed(t); }
    EXPECT_FALSE(d.active(t));
    // and a genuine doubled cadence is still caught afterwards
    for (int i = 0; i < 10; i++) { t += 100; d.feed(t); }
    EXPECT_TRUE(d.active(t));
}

TEST(ZhiannDecode, soc_cadence_ignores_burst_exit_gaps)
{
    // Regression for the ONLY false positive in the 2026-08-13/14 corpus:
    // swap capture, node 1, board time 2584091..2587138ms, 1.7s active on a
    // node carrying a single pack (identity FFFFF6DB throughout).
    //
    // A pack alone on the bus storms retransmits at 4ms because nothing ACKs
    // it. When another device comes up and starts ACKing, the storm breaks
    // into bursts whose EXIT gaps land in the doubled band. MIN_GAP_MS filters
    // the intra-burst 4ms spacing but says nothing about the exit gaps.
    //
    // The 11 scoring gaps below are the real ones, in order, over a 3047ms
    // span. They walk the score 0 -> exactly 10, which is why ACTIVATE is 12.
    SocCadenceDupDetector d;
    const uint32_t scoring[] = {
        70, 118, 176, 53, 77, 135, 237, 458, 58, 490, 57
    };
    uint32_t t = 1000;
    for (uint8_t i = 0; i < sizeof(scoring) / sizeof(scoring[0]); i++) {
        // the burst itself: retransmits below the floor, worth nothing
        for (int j = 0; j < 20; j++) { t += 4; d.feed(t); }
        t += scoring[i];
        d.feed(t);
        EXPECT_FALSE(d.active(t));
    }
    // pin the design decision: this sequence peaks at 10, so any threshold at
    // or below that re-introduces the false positive
    EXPECT_GT(SocCadenceDupDetector::ACTIVATE, 10);
}

TEST(ZhiannDecode, soc_cadence_silence_clears_the_verdict)
{
    // The score only moves when a frame arrives, so a collided pair that goes
    // to deep sleep would otherwise keep reporting a duplicate forever against
    // an empty bus.
    SocCadenceDupDetector d;
    uint32_t t = 1000;
    for (int i = 0; i < 10; i++) { t += 100; d.feed(t); }
    EXPECT_TRUE(d.active(t));
    EXPECT_TRUE(d.active(t + 999));     // still within the silence window
    EXPECT_FALSE(d.active(t + 1000));   // stream gone: no evidence either way
    // and it re-detects cleanly once frames return
    t += 5000;
    for (int i = 0; i < 10; i++) { t += 100; d.feed(t); }
    EXPECT_TRUE(d.active(t));
}

TEST(ZhiannDecode, soc_cadence_tolerates_dropped_frames)
{
    // a single dropped frame doubles the gap; hysteresis must not flap
    SocCadenceDupDetector d;
    uint32_t t = 1000;
    for (int i = 0; i < 10; i++) { t += 100; d.feed(t); }
    EXPECT_TRUE(d.active(t));
    t += 402;               // two missed frames on a collided node
    d.feed(t);
    EXPECT_TRUE(d.active(t));
}

TEST(ZhiannDecode, stdev_of_pack_spread)
{
    // no spread to speak of: a single pack, and identical packs
    EXPECT_FLOAT_EQ(stdev(102.5f, 102.5f * 102.5f, 1), 0.0f);
    float sum = 0, sum_sq = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += 100.0f;
        sum_sq += 100.0f * 100.0f;
    }
    // identical values: the variance subtraction must not go negative and
    // produce a NaN through sqrtf
    EXPECT_FLOAT_EQ(stdev(sum, sum_sq, 4), 0.0f);

    // the four packs measured on the bench 2026-08-14
    const float v[] = { 102.09f, 102.38f, 102.58f, 102.93f };
    sum = 0; sum_sq = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += v[i];
        sum_sq += v[i] * v[i];
    }
    EXPECT_NEAR(stdev(sum, sum_sq, 4), 0.303f, 0.005f);

    // one pack collapsed: the spread is what the operator needs to see
    const float bad[] = { 102.09f, 102.38f, 102.58f, 88.0f };
    sum = 0; sum_sq = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += bad[i];
        sum_sq += bad[i] * bad[i];
    }
    EXPECT_GT(stdev(sum, sum_sq, 4), 6.0f);
}

AP_GTEST_MAIN()
