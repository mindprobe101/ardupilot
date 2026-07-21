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
    if (((id >> 16) & 0xFF) == ALARM_PF) {
        out.node = id & 0xFF;   // SA carries the pack node when known
        out.type = FRAME_ALARM;
        return true;
    }
    return false;
}

inline uint16_t u16(const uint8_t *b) { return uint16_t(b[0] | (b[1] << 8)); }

// PACK_VOLT frame (needs dlc >= 4)
inline float pack_voltage(const uint8_t *d) { return u16(&d[2]) * 0.01f; }

// PACK_VOLT frame current (needs dlc >= 8): s32 LE, 2 mA/LSB, BMS uses
// discharge-negative; returned with ArduPilot's discharge-positive sign
inline float current_amps(const uint8_t *d)
{
    const int32_t raw = int32_t(uint32_t(d[4]) | (uint32_t(d[5]) << 8) |
                                (uint32_t(d[6]) << 16) | (uint32_t(d[7]) << 24));
    return -raw * CURRENT_SCALE;
}

// TEMP_SOC frame (needs dlc >= 6): two signed 0.1C sensors + 0.1% SOC
inline float temp1_c(const uint8_t *d) { return int16_t(u16(&d[0])) * 0.1f; }
inline float temp2_c(const uint8_t *d) { return int16_t(u16(&d[2])) * 0.1f; }
inline uint8_t soc_fine_pct(const uint8_t *d)
{
    const uint16_t v = u16(&d[4]) / 10;
    return v > 100 ? 100 : (uint8_t)v;
}

// SOC_COARSE frame: 1% SOC + pack voltage mirror in 1/320 V
inline uint8_t soc_coarse_pct(const uint8_t *d)
{
    const uint16_t v = u16(&d[0]);
    return v > 100 ? 100 : (uint8_t)v;
}
inline uint16_t soc_voltage_mirror(const uint8_t *d) { return u16(&d[2]); }

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

// duplicate-node detector: a lone pack sends PACK_VOLT every ~500ms, so
// sustained sub-300ms arrivals mean two packs share the node. Hysteresis
// keeps a single glitch from flapping the state
class DupDetector {
public:
    void feed(uint32_t now_ms)
    {
        if (_last_ms != 0) {
            const uint32_t dt = now_ms - _last_ms;
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
private:
    uint32_t _last_ms = 0;
    uint8_t _score = 0;
    bool _active = false;
};

}  // namespace ZhiannBMS
