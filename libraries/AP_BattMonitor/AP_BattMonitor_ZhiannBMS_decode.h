/*
  pure frame classification and decoding for the Zhiann CAN BMS drone
  broadcast profile. No HAL or driver dependencies so it is usable from
  both AP_BattMonitor_ZhiannBMS and desktop unit tests replaying real
  captured frames. Protocol details: see the driver's header comment.
 */
#pragma once

#include <stdint.h>

namespace ZhiannBMS {

static const uint32_t BLOCK_BASE   = 0x2E0941UL;  // + 0x20 * node
static const uint32_t SOC_BASE     = 0x401A100UL; // + node
static const uint32_t PACKID_BASE  = 0x402A100UL; // + node, 2s identity frame
static const uint8_t  MAX_NODE     = 15;
static const uint8_t  ALARM_PF     = 0x24;
static const uint8_t  CELL_SLICE_COUNT = 7;
// Current scale, amps per LSB of the s32 in the PACK_VOLT frame.
// 1 mA/LSB, established 2026-08-14: the pack broadcasts its own rated
// capacity of 44.0 Ah, and coulomb-counting 14 pack-flights against the
// packs' own SOC gives ~82 Ah apparent at 2 mA/LSB. The original 2 mA/LSB
// came from correlating against an analog power module whose own
// calibration was the weak link. BATTn_CURR_MULT trims residual error.
static const float    CURRENT_SCALE = 0.001f;

// on-wire block frame types (low 5 bits of id - BLOCK_BASE) plus
// internal tags outside the 5-bit range
enum FrameType : uint8_t {
    FRAME_CELL_19_22 = 0x00,
    FRAME_TEMP_SOC   = 0x01,
    FRAME_CELL_1_2   = 0x02,
    FRAME_CELL_3_6   = 0x03,
    FRAME_CELL_7_10  = 0x04,
    FRAME_CELL_11_14 = 0x05,
    FRAME_CELL_15_18 = 0x06,
    FRAME_CELL_23_24 = 0x09,
    FRAME_PACK_VOLT  = 0x10,
    FRAME_PACK_ID    = 0xFD,
    FRAME_ALARM      = 0xFE,
    FRAME_SOC_COARSE = 0xFF,
};

struct Classified {
    uint8_t node;
    uint8_t type;
};

// classify a 29-bit extended id into pack node + frame type; only known
// frame types are accepted so unrelated devices in the extrapolated ID
// range are never consumed or allowed to trigger instance binding
inline bool classify(uint32_t id, Classified &out)
{
    if (id >= BLOCK_BASE && id < BLOCK_BASE + 0x20UL * (MAX_NODE + 1)) {
        out.node = (id - BLOCK_BASE) >> 5;
        const uint8_t t = (id - BLOCK_BASE) & 0x1F;
        switch (t) {
        case FRAME_CELL_19_22:
        case FRAME_TEMP_SOC:
        case FRAME_CELL_1_2:
        case FRAME_CELL_3_6:
        case FRAME_CELL_7_10:
        case FRAME_CELL_11_14:
        case FRAME_CELL_15_18:
        case FRAME_CELL_23_24:
        case FRAME_PACK_VOLT:
            out.type = t;
            return true;
        default:
            return false;
        }
    }
    if ((id & ~0xFUL) == SOC_BASE) {
        out.node = id & 0xF;
        out.type = FRAME_SOC_COARSE;
        return true;
    }
    // 2s frame carrying a per-pack identity in bytes 4..7. Emitted in
    // standby as well as ON, so a duplicate node is detectable before the
    // packs are even switched on.
    if ((id & ~0xFUL) == PACKID_BASE) {
        out.node = id & 0xF;
        out.type = FRAME_PACK_ID;
        return true;
    }
    // Vendor-standard alarm PGN: any priority, R/DP zero, PF 0x24,
    // destination either the null address 0x00 or the controller class
    // 0xF0..0xFF, BMS source address 0x00..0xEF (including the vendor
    // protocol's random-address range 0x80..0xEF). Alarm delivery must not
    // hinge on unverified priority or exact-destination assumptions.
    // The source address is not necessarily the proprietary broadcast node.
    const uint8_t ps = (id >> 8) & 0xFF;
    const uint8_t sa = id & 0xFF;
    if ((id & 0x03FF0000UL) == 0x00240000UL &&
        (ps == 0x00 || ps >= 0xF0) && sa <= 0xEF) {
        out.node = sa;
        out.type = FRAME_ALARM;
        return true;
    }
    return false;
}

inline uint16_t u16(const uint8_t *b) { return uint16_t(b[0] | (b[1] << 8)); }

// Captures contain one canonical DLC for every accepted type. Validate this
// before a frame is allowed to bind an instance or refresh any state.
inline uint8_t expected_dlc(uint8_t frame_type)
{
    switch (frame_type) {
    case FRAME_CELL_23_24:
        return 4;
    case FRAME_CELL_19_22:
    case FRAME_TEMP_SOC:
    case FRAME_CELL_1_2:
    case FRAME_CELL_3_6:
    case FRAME_CELL_7_10:
    case FRAME_CELL_11_14:
    case FRAME_CELL_15_18:
    case FRAME_PACK_VOLT:
    case FRAME_SOC_COARSE:
    case FRAME_PACK_ID:
        return 8;
    default:
        return 0;
    }
}

// Per-pack identity from the 2s frame (needs dlc >= 8), stable per pack and
// available without a polling master. Read in wire order rather than
// little-endian like the numeric fields, so the printed value matches the
// byte order seen in captures (FFFF5B8C, FFFFBDBD, ...).
inline uint32_t pack_id(const uint8_t *d)
{
    return (uint32_t(d[4]) << 24) | (uint32_t(d[5]) << 16) |
           (uint32_t(d[6]) << 8) | uint32_t(d[7]);
}

// Duplicate detection on the SOC-coarse stream.
//
// This is the primary duplicate detector because every pack emits this frame
// on its own node, and it doubles while the second pack is still in standby -
// roughly 150s before the detail frames double. Measured over the 2026-08-13
// captures: one pack 201ms median (4.5/s), two packs 70-104ms (9/s).
//
// Intervals below MIN_GAP_MS are ignored. A node with nothing else on the bus
// to ACK it retransmits every frame at ~4ms, and the CAN RX queue drains
// back-to-back at startup; both look like an impossibly fast cadence and
// caused the historical boot-time false duplicates. Validated over the corpus:
// 70,922 sub-20ms gaps, of which 69,057 are the 4ms no-ACK retransmit and the
// largest genuine queue-drain gap is 19ms. Dropping the floor to 0 produces
// 433s of false activity, so it is load-bearing.
//
// ACTIVATE is 12, not 10. At 10 the corpus contains one real false positive:
// a single pack alone on the bus was storming retransmits, another device came
// up and began ACKing, and the burst-exit gaps (70, 118, 53, 77, 135, 58,
// 57ms) scored seven +2 against one -1 and tripped at exactly 10. That
// transition - an FC powering up on a bus where a pack is already storming -
// is the normal power-on order for the aircraft, so it is not a corner case.
// At 12 it does not trip, the worst single-pack excursion anywhere else in
// 3.74h of single-pack node time is 4, and the cost is 0.09% of duplicate-time
// coverage and ~0.2s of onset latency.
//
// Note the thresholds do not separate the two distributions with margin: 0.9%
// of single-pack gaps land in the doubled band. What makes this safe is the
// 2:1 scoring ratio, which requires a tight CLUSTER of doubled gaps rather
// than any single one. The margin is statistical, not deterministic.
class SocCadenceDupDetector {
public:
    static const uint32_t MIN_GAP_MS = 20;    // below: retransmit/queue drain
    static const uint32_t DOUBLED_MS = 150;   // at or below: two packs
    static const uint32_t SINGLE_MS  = 160;   // at or above: one pack
    static const uint32_t SILENCE_MS = 1000;  // no frames at all: no evidence
    static const uint8_t  SCORE_MAX  = 20;
    static const uint8_t  ACTIVATE   = 12;
    static const uint8_t  DEACTIVATE = 3;

    void feed(uint32_t now_ms)
    {
        if (_last_ms != 0) {
            const uint32_t dt = now_ms - _last_ms;
            if (dt < MIN_GAP_MS) {
                // artifact, carries no cadence information
            } else if (dt <= DOUBLED_MS) {
                _score = _score >= SCORE_MAX - 2 ? SCORE_MAX : _score + 2;
            } else if (dt >= SINGLE_MS && _score > 0) {
                _score--;
            }
        }
        _last_ms = now_ms;
        _seen_ms = now_ms;
    }

    // hysteresis stops a single dropped frame flapping the verdict
    bool active(uint32_t now_ms)
    {
        // The score only moves when a frame arrives, so without this the
        // verdict freezes forever once the stream stops - a collided pair that
        // goes to deep sleep would keep reporting a duplicate against an empty
        // bus. Silence is not evidence of anything, so drop the verdict.
        //
        // Keyed on frame ARRIVAL, not on scoring events: a surviving pack left
        // alone and storming retransmits produces no scoring gaps at all, but
        // it is still transmitting and the last cadence verdict is still the
        // best evidence there is. That case clears the slow way (up to ~34s),
        // which is a nuisance but errs toward blocking.
        if (_seen_ms != 0 && now_ms - _seen_ms >= SILENCE_MS) {
            reset();
        }
        if (!_active && _score >= ACTIVATE) {
            _active = true;
        } else if (_active && _score <= DEACTIVATE) {
            _active = false;
        }
        return _active;
    }

    void reset() { _last_ms = 0; _seen_ms = 0; _score = 0; _active = false; }

private:
    uint32_t _last_ms = 0;
    uint32_t _seen_ms = 0;  // last frame of any spacing, incl. retransmits
    uint8_t _score = 0;
    bool _active = false;
};

// Corroborating detector: names the two packs when it can see them.
//
// NOT a general duplicate detector. Not every occupied node emits the 2s
// identity frame, and the rule governing which do is not known: across the
// 2026-08-13/14 captures node 0 emitted it every time, other nodes emitted it
// inconsistently, and in one capture three nodes emitted concurrently while an
// occupied fourth stayed silent. What matters here is that two of the three
// real collisions in the corpus produced no identity frames at all, so a
// collision is usually invisible to this detector. Measured over the corpus:
// zero false positives across ~4700s of single-pack node time, but only ~20%
// of real duplicate time detected. Use SocCadenceDupDetector for detection and
// this only to identify which packs are involved.
class IdentityDupDetector {
public:
    void feed(uint32_t id, uint32_t now_ms)
    {
        if (id == 0 || id == 0xFFFFFFFFUL) {
            return;                     // sentinel / not yet populated
        }
        if (_has_a && id == _id_a) { _seen_a = now_ms; return; }
        if (_has_b && id == _id_b) { _seen_b = now_ms; return; }
        if (!_has_a) { _id_a = id; _seen_a = now_ms; _has_a = true; return; }
        if (!_has_b) { _id_b = id; _seen_b = now_ms; _has_b = true; return; }
        // both slots taken by other ids: evict whichever was seen longer ago
        if (uint32_t(now_ms - _seen_a) >= uint32_t(now_ms - _seen_b)) {
            _id_a = id; _seen_a = now_ms;
        } else {
            _id_b = id; _seen_b = now_ms;
        }
    }

    // two different packs have both been heard recently on this node
    bool active(uint32_t now_ms, uint32_t window_ms = 6000) const
    {
        return _has_a && _has_b &&
               uint32_t(now_ms - _seen_a) <= window_ms &&
               uint32_t(now_ms - _seen_b) <= window_ms;
    }

    // the two contending ids, for the operator-facing message
    uint32_t id_a() const { return _id_a; }
    uint32_t id_b() const { return _id_b; }
    // most recently heard id, i.e. whoever last spoke for this node
    uint32_t latest_id() const
    {
        if (!_has_a) { return 0; }
        if (!_has_b) { return _id_a; }
        return uint32_t(_seen_a - _seen_b) < 0x80000000UL ? _id_a : _id_b;
    }
    void reset()
    {
        _has_a = _has_b = false;
        _id_a = _id_b = 0;
        _seen_a = _seen_b = 0;
    }

private:
    uint32_t _id_a = 0, _id_b = 0;
    uint32_t _seen_a = 0, _seen_b = 0;
    bool _has_a = false, _has_b = false;
};

inline bool valid_dlc(uint8_t frame_type, uint8_t dlc)
{
    if (frame_type == FRAME_ALARM) {
        // both alarm words live in bytes 0-3; a shorter-than-spec alarm
        // frame must still be heard rather than dropped on frame length
        return dlc >= 4 && dlc <= 8;
    }
    const uint8_t expected = expected_dlc(frame_type);
    return expected != 0 && dlc == expected;
}

// PACK_VOLT frame (canonical dlc 8)
inline float pack_voltage(const uint8_t *d) { return u16(&d[2]) * 0.01f; }

// PACK_VOLT frame current (needs dlc >= 8): s32 LE, 1 mA/LSB, BMS uses
// discharge-negative; returned with ArduPilot's discharge-positive sign
inline float current_amps(const uint8_t *d)
{
    const int32_t raw = int32_t(uint32_t(d[4]) | (uint32_t(d[5]) << 8) |
                                (uint32_t(d[6]) << 16) | (uint32_t(d[7]) << 24));
    // Convert before negation so raw INT32_MIN remains well-defined.
    return -float(raw) * CURRENT_SCALE;
}

// Deliberately broad limits: reject corrupt/sentinel values without
// constraining the normal operating envelope of a 24S propulsion pack.
inline bool pack_voltage_valid(float voltage) { return voltage >= 1.0f && voltage <= 200.0f; }
inline bool current_valid(float current) { return current >= -10000.0f && current <= 10000.0f; }

// TEMP_SOC frame (needs dlc >= 6): two signed 0.1C sensors + 0.1% SOC
inline float temp1_c(const uint8_t *d) { return int16_t(u16(&d[0])) * 0.1f; }
inline float temp2_c(const uint8_t *d) { return int16_t(u16(&d[2])) * 0.1f; }
inline bool temperature_valid(float temperature) { return temperature >= -100.0f && temperature <= 200.0f; }
inline uint16_t soc_fine_tenths(const uint8_t *d)
{
    return u16(&d[4]);
}
inline uint8_t soc_fine_pct(const uint8_t *d) { return soc_fine_tenths(d) / 10; }
inline bool soc_fine_valid(const uint8_t *d) { return soc_fine_tenths(d) <= 1000; }

// SOC_COARSE frame: 1% SOC + pack voltage mirror in 1/320 V
inline uint8_t soc_coarse_pct(const uint8_t *d)
{
    // Byte 0 bit 7 and byte 1 are status/dynamic data. Load and standby
    // captures prove only the low seven bits carry the 0..100 percentage.
    const uint8_t v = d[0] & 0x7F;
    return v;
}
inline bool soc_coarse_valid(const uint8_t *d) { return (d[0] & 0x7F) <= 100; }
inline uint16_t soc_voltage_mirror(const uint8_t *d) { return u16(&d[2]); }

inline bool pack_vmir_coherent(float pack_voltage, uint16_t mirror_raw,
                               float tolerance_v)
{
    float difference_v = mirror_raw / 320.0f - pack_voltage;
    if (difference_v < 0) {
        difference_v = -difference_v;
    }
    return difference_v <= tolerance_v;
}

inline bool fresh_ms(uint32_t now_ms, uint32_t sample_ms, uint32_t timeout_ms)
{
    return sample_ms != 0 && (now_ms - sample_ms) <= timeout_ms;
}

inline uint32_t consumption_dt_us(uint64_t elapsed_us)
{
    // update_consumed() rejects values >=2 seconds. Saturating prevents a
    // multi-hour outage from folding through uint32_t and looking recent.
    return elapsed_us > UINT32_MAX ? UINT32_MAX : uint32_t(elapsed_us);
}

inline float consumed_mah_from_soc(float capacity_mah, uint16_t soc_tenths)
{
    const float bounded_soc = soc_tenths > 1000 ? 1000.0f : float(soc_tenths);
    return capacity_mah * (1000.0f - bounded_soc) * 0.001f;
}

inline float reconciled_consumed_mah(float integrated_mah, float capacity_mah,
                                     uint16_t soc_tenths)
{
    // Monotonic floor: SOC-derived consumption may raise the integrated
    // total but never lower it, including when the integrator re-seeds
    // after a session reset.
    const float soc_mah = consumed_mah_from_soc(capacity_mah, soc_tenths);
    return soc_mah > integrated_mah ? soc_mah : integrated_mah;
}

// choose the hotter of the plausible sensors; false when both are implausible
inline bool select_temperature(float t1, float t2, float &out)
{
    const bool t1_ok = temperature_valid(t1);
    const bool t2_ok = temperature_valid(t2);
    if (t1_ok && t2_ok) {
        out = t1 > t2 ? t1 : t2;
    } else if (t1_ok) {
        out = t1;
    } else if (t2_ok) {
        out = t2;
    } else {
        return false;
    }
    return true;
}

inline bool pack_cells_coherent(float pack_voltage, const uint16_t *cells,
                                uint8_t cell_count, float tolerance_v)
{
    uint32_t sum_mv = 0;
    for (uint8_t i = 0; i < cell_count; i++) {
        if (cells[i] < 500 || cells[i] > 6000) {
            return false;
        }
        sum_mv += cells[i];
    }
    float difference_v = sum_mv * 0.001f - pack_voltage;
    if (difference_v < 0) {
        difference_v = -difference_v;
    }
    return difference_v <= tolerance_v;
}

// cell-voltage frame map: frame type -> first cell index (0-based),
// payload byte offset, cell count
struct CellSlice {
    uint8_t frame_type;
    uint8_t first_cell;
    uint8_t offset;
    uint8_t ncells;
};
static const CellSlice CELL_MAP[] = {
    { FRAME_CELL_1_2,    0, 4, 2 },
    { FRAME_CELL_3_6,    2, 0, 4 },
    { FRAME_CELL_7_10,   6, 0, 4 },
    { FRAME_CELL_11_14, 10, 0, 4 },
    { FRAME_CELL_15_18, 14, 0, 4 },
    { FRAME_CELL_19_22, 18, 0, 4 },
    { FRAME_CELL_23_24, 22, 0, 2 },
};

// Assemble cell frames into a private buffer and expose them only when all
// seven canonical slices arrive in one burst. This prevents main-thread
// readers from seeing a mixture while the next 500ms generation is arriving.
class CellAccumulator {
public:
    bool feed(uint8_t frame_type, const uint8_t *data, uint32_t now_ms,
              uint32_t max_span_ms)
    {
        uint8_t slice = 0;
        for (; slice < CELL_SLICE_COUNT; slice++) {
            if (CELL_MAP[slice].frame_type == frame_type) {
                break;
            }
        }
        if (slice == CELL_SLICE_COUNT) {
            return false;
        }

        // CELL_1_2 is the first cell frame in every observed broadcast burst.
        if (slice == 0) {
            reset();
            _start_ms = now_ms;
            _cell_count = u16(&data[2]);
        } else if (_slice_mask == 0) {
            return false;
        }

        if ((now_ms - _start_ms) > max_span_ms) {
            reset();
            return false;
        }
        if (slice != _next_slice) {
            // Duplicate or out-of-order slices are ambiguous when two packs
            // share a node. Discard the pending generation fail-safe.
            reset();
            return false;
        }

        const CellSlice &m = CELL_MAP[slice];
        for (uint8_t i = 0; i < m.ncells; i++) {
            _cells[m.first_cell + i] = u16(&data[m.offset + 2 * i]);
        }
        _slice_mask |= uint8_t(1U << slice);
        _next_slice++;
        return _cell_count == 24 &&
               _slice_mask == uint8_t((1U << CELL_SLICE_COUNT) - 1U);
    }

    void reset()
    {
        _start_ms = 0;
        _cell_count = 0;
        _slice_mask = 0;
        _next_slice = 0;
    }

    const uint16_t *cells() const { return _cells; }
    uint16_t cell_count() const { return _cell_count; }

private:
    uint16_t _cells[24] {};
    uint32_t _start_ms = 0;
    uint16_t _cell_count = 0;
    uint8_t _slice_mask = 0;
    uint8_t _next_slice = 0;
};

// duplicate-node detector: a lone pack sends PACK_VOLT every ~500ms, so
// sustained sub-300ms arrivals mean two packs share the node. Hysteresis
// keeps a single glitch from flapping the state
class DupDetector {
public:
    // Same retransmit/queue-drain floor as SocCadenceDupDetector, and for the
    // same reason. Without it the first pack to appear on a quiet bus emits a
    // startup burst of PACK_VOLT frames whose sub-millisecond spacing scores
    // as an impossibly fast cadence: observed on the bench 2026-08-14, where
    // the first of four packs tripped a false duplicate on node 0 within one
    // second and held it 9.5s (decay here is one point per ~508ms detail
    // frame). The packs that joined 2s later onto an already-busy bus showed
    // clean 508ms cadence from their first frame and never tripped.
    //
    // This is the NORMAL case on this aircraft, not an edge case: the Cube
    // runs on a separate avionics battery, so it is already on the bus when
    // the packs are switched on, and it therefore sees every pack's storm.
    //
    // DO NOT RAISE THIS FLOOR. Two packs on one node do not interleave evenly
    // at ~254ms; they hold a locked, small phase offset and alternate a short
    // gap with a long one. Measured over the corpus, the short gaps have
    // median 40ms and minimum 36ms, so 20ms sits only 16ms below the real
    // collision signature. Replay confirms it: at 20ms the floor removes 5 of
    // the 6 single-pack false positives (~10s each) while all three genuine
    // collisions are still detected to within 0.5s of their full duration.
    //
    // The sixth is a pack handover on an occupied node, where the data really
    // is a mixture while it happens; raising the activation threshold to 16
    // does not remove it, so it is left to the settling window to keep quiet.
    static const uint32_t MIN_GAP_MS = 20;

    void feed(uint32_t now_ms)
    {
        if (_last_ms != 0) {
            const uint32_t dt = now_ms - _last_ms;
            if (dt >= 400 && dt <= 750) {
                if (_clean_intervals < 5) {
                    _clean_intervals++;
                }
            } else if (dt > 750 && dt <= 1800) {
                // up to ~3 lost ~500ms frames: neither collision evidence
                // nor a clean interval, so hold qualification progress.
                // Long gaps are never a collision signature; resetting on
                // them only makes qualification flap during dropouts
            } else {
                _clean_intervals = 0;
            }
            if (dt >= MIN_GAP_MS && dt < 300) {
                _score = _score >= 18 ? 20 : _score + 2;
            } else if (dt >= 400 && _score > 0) {
                _score--;
            }
        }
        _last_ms = now_ms;
    }
    bool active()
    {
        if (!_active && _score >= 10) {
            _active = true;
        } else if (_active && _score <= 3) {
            _active = false;
        }
        return _active;
    }
    bool qualified() const { return _clean_intervals >= 5; }
    void reset_qualification() { _clean_intervals = 0; }
private:
    uint32_t _last_ms = 0;
    uint8_t _score = 0;
    uint8_t _clean_intervals = 0;
    bool _active = false;
};

// Two consecutive mismatches from the same evidence stream (SOC voltage
// mirror or cell-sum) are strong collision evidence and activate the fault;
// a lone mismatch (bus glitch, boundary sample) only arms that stream's
// pending strike, which the stream's next clean check clears. The streams
// keep separate strikes: one physical transient hitting both checks once
// each within a burst must not activate. Once active, require a run of
// clean independent checks before clearing so alternating mixed and coherent
// frames cannot make health flicker during cadence-detector warm-up.
class CoherenceDetector {
public:
    void feed_vmir(bool coherent) { feed(_pending_vmir, coherent); }
    void feed_cellsum(bool coherent) { feed(_pending_cellsum, coherent); }
    bool active() const { return _score > 0; }

    // session boundary (>5s outage): a half-armed strike must not pair
    // with the new session's first mismatch, while an accumulated active
    // score is deliberately preserved as evidence
    void clear_pending()
    {
        _pending_vmir = false;
        _pending_cellsum = false;
    }

private:
    void feed(bool &pending, bool coherent)
    {
        if (!coherent) {
            if (pending) {
                _score = 20;
            }
            pending = true;
        } else {
            pending = false;
            if (_score > 0) {
                _score--;
            }
        }
    }

    uint8_t _score = 0;
    bool _pending_vmir = false;
    bool _pending_cellsum = false;
};

}  // namespace ZhiannBMS
