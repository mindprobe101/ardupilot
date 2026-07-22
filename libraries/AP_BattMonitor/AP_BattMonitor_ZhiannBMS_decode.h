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
static const uint8_t  MAX_NODE     = 15;
static const uint8_t  ALARM_PF     = 0x24;
static const uint8_t  CELL_SLICE_COUNT = 7;
// nominal current scale, amps per LSB of the s32 in the PACK_VOLT frame
static const float    CURRENT_SCALE = 0.002f;

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
    // Vendor-standard alarm PGN: priority 6, R/DP zero, PF 0x24,
    // controller destination 0xF0..0xFE, BMS source address 0x00..0xEF
    // (including the vendor protocol's random-address range 0x80..0xEF).
    // The source address is not necessarily the proprietary broadcast node.
    const uint8_t ps = (id >> 8) & 0xFF;
    const uint8_t sa = id & 0xFF;
    if ((id & 0x1FFF0000UL) == 0x18240000UL &&
        ps >= 0xF0 && ps <= 0xFE && sa <= 0xEF) {
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
    case FRAME_ALARM:
    case FRAME_SOC_COARSE:
        return 8;
    default:
        return 0;
    }
}

inline bool valid_dlc(uint8_t frame_type, uint8_t dlc)
{
    const uint8_t expected = expected_dlc(frame_type);
    return expected != 0 && dlc == expected;
}

// PACK_VOLT frame (canonical dlc 8)
inline float pack_voltage(const uint8_t *d) { return u16(&d[2]) * 0.01f; }

// PACK_VOLT frame current (needs dlc >= 8): s32 LE, 2 mA/LSB, BMS uses
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
                                     uint16_t soc_tenths, bool already_seeded)
{
    const float soc_mah = consumed_mah_from_soc(capacity_mah, soc_tenths);
    return !already_seeded || soc_mah > integrated_mah ? soc_mah : integrated_mah;
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
    void feed(uint32_t now_ms)
    {
        if (_last_ms != 0) {
            const uint32_t dt = now_ms - _last_ms;
            if (dt >= 400 && dt <= 750) {
                if (_clean_intervals < 5) {
                    _clean_intervals++;
                }
            } else {
                _clean_intervals = 0;
            }
            if (dt < 300) {
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

// A mismatch is strong collision evidence and activates immediately. Require
// a run of clean independent checks before clearing so alternating mixed and
// coherent frames cannot make health flicker during cadence-detector warm-up.
class CoherenceDetector {
public:
    void feed(bool coherent)
    {
        if (!coherent) {
            _score = 20;
        } else if (_score > 0) {
            _score--;
        }
    }
    bool active() const { return _score > 0; }

private:
    uint8_t _score = 0;
};

}  // namespace ZhiannBMS
