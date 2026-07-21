/*
  battery monitor for the Zhiann CAN BMS

  protocol reverse engineered from live packs (2026-07): 1 Mbit/s classic
  CAN, all IDs 29-bit extended, values little-endian u16 unless noted.

  Multiple packs share one bus. Pack n transmits the ID block
  0x2E0941 + 0x20*n plus an SOC frame at 0x401A100 + n (4 packs observed
  live; the ID arithmetic is extrapolated up to node 15, the SOC frame's
  node nibble - ArduPilot itself supports 9 monitors). Node numbering
  follows hub/chain position. Frame types within a pack block (offset
  from 0x...41):

    +0x00  cell voltages 19..22 (mV)
    +0x01  temperature1 (0.1 C), temperature2 (0.1 C), SOC (0.1 %)
    +0x02  unknown, cell count, cell voltages 1..2 (mV)
    +0x03  cell voltages 3..6 (mV)
    +0x04  cell voltages 7..10 (mV)
    +0x05  cell voltages 11..14 (mV)
    +0x06  cell voltages 15..18 (mV)
    +0x09  cell voltages 23..24 (mV)
    +0x10  counter/crc, pack voltage (10 mV), current (s32, 2 mA,
           negative = discharge, reads 0 below a few amps)

  0x401A100+n: SOC (%), pack voltage mirror (1/320 V), temperature?
  0x402A100: single transmitter on the bus, unused here.

  The pack block repeats every ~500 ms. The nominal current scale was
  calibrated against an ArduPilot analog power module (steady-load
  plateaus 13..48 A gave 2.05 +/- 0.05 mA/LSB -> 2 mA nominal); use
  BATTn_CURR_MULT to trim in the field.

  Instance selection: set BATTn_SERIAL_NUM to the pack node (0..15), or
  leave -1 to bind to the first pack not claimed by any other instance.
  Auto-binding is resolved in BATTn instance order, independent of CAN
  callback registration order.

  Limitations:
   - one bus only: MultiCAN merges all ports configured with this
     protocol into one stream with no port identity, so identically
     numbered packs on two buses would alias. Configure the protocol on
     a single CAN port.
   - CAN rangefinders with RNGFNDx_RECV_ID=0 consume frames of any ID
     via the shared MultiCAN dispatch list and can starve this driver;
     set an explicit RECV_ID when combining them with this BMS.
 */
#include "AP_BattMonitor_ZhiannBMS.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#include <string.h>

// pack n block: 0x2E0941 + 0x20*n, frame type = low 5 bits of (id - 0x41)
#define ZHIANN_BLOCK_BASE      0x2E0941UL
#define ZHIANN_SOC_BASE        0x401A100UL
// highest pack node: bounded by the node nibble of the SOC frame ID
#define ZHIANN_MAX_NODE        15

// frame types within a pack block (0..0x1F on the wire)
#define ZHIANN_FRAME_CELL_19_22   0x00
#define ZHIANN_FRAME_TEMP_SOC     0x01
#define ZHIANN_FRAME_CELL_1_2     0x02
#define ZHIANN_FRAME_CELL_3_6     0x03
#define ZHIANN_FRAME_CELL_7_10    0x04
#define ZHIANN_FRAME_CELL_11_14   0x05
#define ZHIANN_FRAME_CELL_15_18   0x06
#define ZHIANN_FRAME_CELL_23_24   0x09
#define ZHIANN_FRAME_PACK_VOLT    0x10
// internal tags; outside the 5-bit wire range so they can never collide
// with a real block frame type
#define ZHIANN_FRAME_SOC_COARSE   0xFF
#define ZHIANN_FRAME_ALARM        0xFE

// spec J1939-style frames: id = P<<26 | PF<<16 | PS<<8 | SA
#define ZHIANN_PF(id)             (((id) >> 16) & 0xFFU)
#define ZHIANN_PF_ALARM           0x24U

// NOTE: do NOT transmit the spec master heartbeat (PF 0x43) to these
// packs. Field-tested 2026-07-21: a continuous heartbeat whose
// registered-bitmap omits a momentarily-silent pack makes that pack
// release its node address and re-join on an already-taken node,
// colliding IDs with another pack; the packs also power-managed
// themselves off when heartbeats appeared. The drone-profile packs
// broadcast fine without any heartbeat.

// alarm frame bit definitions (same layout in alarm and warning words)
#define ZHIANN_ALARM_CHG_OVERCURRENT   (1U << 0)
#define ZHIANN_ALARM_TEMP_SENSOR       (1U << 8)
#define ZHIANN_ALARM_CELL_OVERVOLT     (1U << 9)
#define ZHIANN_ALARM_CELL_UNDERVOLT    (1U << 10)
#define ZHIANN_ALARM_HIGH_TEMP         (1U << 11)
#define ZHIANN_ALARM_LOW_TEMP          (1U << 12)
#define ZHIANN_ALARM_AFE_FAULT         (1U << 13)
#define ZHIANN_ALARM_BATT_DAMAGED      (1U << 14)
#define ZHIANN_ALARM_DIS_OVERCURRENT   (1U << 15)
// alarms that make the battery unsafe to arm on
#define ZHIANN_ALARM_SEVERE  (ZHIANN_ALARM_BATT_DAMAGED | ZHIANN_ALARM_AFE_FAULT)

// consider the BMS gone after the codebase-wide battery timeout
#define ZHIANN_TIMEOUT_US      (AP_BATT_MONITOR_TIMEOUT * 1000ULL)

// accept the coarse 1% SOC only when the fine 0.1% frame (nominally
// every 500ms) has been silent this long
#define ZHIANN_FINE_SOC_STALE_MS  2500

// nominal current scale, amps per LSB of the s32 in the pack voltage
// frame (discharge negative); trimmed by BATTn_CURR_MULT
#define ZHIANN_CURRENT_SCALE   0.002f

const AP_Param::GroupInfo AP_BattMonitor_ZhiannBMS::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to the BMS reported current, to trim the nominal 2mA/LSB scale against a calibrated reference
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_ZhiannBMS, _curr_mult, 1.0),

    // Param indexes must be between 30 and 35 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

MultiCAN *AP_BattMonitor_ZhiannBMS::_multican;
AP_BattMonitor_ZhiannBMS *AP_BattMonitor_ZhiannBMS::_instances[AP_BATT_MONITOR_MAX_INSTANCES];
uint8_t AP_BattMonitor_ZhiannBMS::_num_instances;

// cell-voltage frame map: frame type -> first cell, payload offset, count
static const struct {
    uint8_t frame_type;
    uint8_t first_cell;
    uint8_t offset;
    uint8_t ncells;
} zhiann_cell_map[] = {
    { ZHIANN_FRAME_CELL_1_2,    0, 4, 2 },
    { ZHIANN_FRAME_CELL_3_6,    2, 0, 4 },
    { ZHIANN_FRAME_CELL_7_10,   6, 0, 4 },
    { ZHIANN_FRAME_CELL_11_14, 10, 0, 4 },
    { ZHIANN_FRAME_CELL_15_18, 14, 0, 4 },
    { ZHIANN_FRAME_CELL_19_22, 18, 0, 4 },
    { ZHIANN_FRAME_CELL_23_24, 22, 0, 2 },
};

AP_BattMonitor_ZhiannBMS::AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    // cells not yet received report as not-present
    memset(_interim_state.cell_voltages.cells, 0xFF,
           sizeof(_interim_state.cell_voltages.cells));

    // starts with not healthy
    _state.healthy = false;

    // register in the shared dispatch table; instances are created in
    // BATTn order by the front-end, which fixes the auto-bind priority
    if (_num_instances < ARRAY_SIZE(_instances)) {
        _instances[_num_instances++] = this;
    }
    if (_multican == nullptr) {
        _multican = NEW_NOTHROW MultiCAN{
            FUNCTOR_BIND_MEMBER(&AP_BattMonitor_ZhiannBMS::dispatch_frame,
                                bool, AP_HAL::CANFrame &),
            AP_CAN::Protocol::ZhiannBMS, "ZhiannBMS"};
        if (_multican == nullptr) {
            AP_BoardConfig::allocation_error("ZhiannBMS multican");
        }
    }
}

bool AP_BattMonitor_ZhiannBMS::matches_node(uint8_t node) const
{
    const int32_t serial = _params._serial_number.get();
    if (serial >= 0) {
        return node == (uint8_t)serial;
    }
    return _auto_node >= 0 && node == (uint8_t)_auto_node;
}

bool AP_BattMonitor_ZhiannBMS::node_claimed(uint8_t node)
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        if (_instances[i]->_params._serial_number.get() == (int32_t)node) {
            return true;
        }
    }
    return false;
}

// shared MultiCAN callback (CAN driver thread), bound to the first
// instance but dispatching across all of them
bool AP_BattMonitor_ZhiannBMS::dispatch_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame()) {
        return false;
    }
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    // BMS alarm frame (PF 0x24, spec 6.8): never observed on the healthy
    // bench bus, listened for opportunistically. SA carries the pack node
    // in the broadcast profile; an unattributable alarm goes to every
    // instance - over-reporting a battery fault beats missing one
    if (ZHIANN_PF(id) == ZHIANN_PF_ALARM && frame.dlc >= 4) {
        const uint8_t sa = id & 0xFF;
        bool attributed = false;
        for (uint8_t i = 0; i < _num_instances; i++) {
            if (_instances[i]->matches_node(sa)) {
                _instances[i]->handle_pack_frame(ZHIANN_FRAME_ALARM, frame);
                attributed = true;
            }
        }
        if (!attributed) {
            for (uint8_t i = 0; i < _num_instances; i++) {
                _instances[i]->handle_pack_frame(ZHIANN_FRAME_ALARM, frame);
            }
        }
        return true;
    }

    uint8_t node;
    uint8_t frame_type;
    if (id >= ZHIANN_BLOCK_BASE &&
        id < ZHIANN_BLOCK_BASE + 0x20UL * (ZHIANN_MAX_NODE + 1)) {
        node = (id - ZHIANN_BLOCK_BASE) >> 5;
        frame_type = (id - ZHIANN_BLOCK_BASE) & 0x1F;
        // only accept frame types we know; unknown types are neither
        // consumed nor allowed to trigger auto-binding, so unrelated
        // devices in the extrapolated ID range are left alone
        switch (frame_type) {
        case ZHIANN_FRAME_CELL_19_22:
        case ZHIANN_FRAME_TEMP_SOC:
        case ZHIANN_FRAME_CELL_1_2:
        case ZHIANN_FRAME_CELL_3_6:
        case ZHIANN_FRAME_CELL_7_10:
        case ZHIANN_FRAME_CELL_11_14:
        case ZHIANN_FRAME_CELL_15_18:
        case ZHIANN_FRAME_CELL_23_24:
        case ZHIANN_FRAME_PACK_VOLT:
            break;
        default:
            return false;
        }
    } else if ((id & ~0xFUL) == ZHIANN_SOC_BASE) {
        node = id & 0xF;
        frame_type = ZHIANN_FRAME_SOC_COARSE;
    } else {
        return false;
    }

    // pass 1: an instance already bound to this node (explicit serial
    // or previously auto-bound)
    for (uint8_t i = 0; i < _num_instances; i++) {
        if (_instances[i]->matches_node(node)) {
            _instances[i]->handle_pack_frame(frame_type, frame);
            return true;
        }
    }

    // pass 2: first unbound auto instance, in BATTn order, unless the
    // node is explicitly claimed by a not-yet-created instance's serial
    if (!node_claimed(node)) {
        for (uint8_t i = 0; i < _num_instances; i++) {
            AP_BattMonitor_ZhiannBMS &inst = *_instances[i];
            if (inst._params._serial_number.get() < 0 && inst._auto_node < 0) {
                inst._auto_node = (int8_t)node;
                inst.handle_pack_frame(frame_type, frame);
                return true;
            }
        }
    }
    return false;
}

void AP_BattMonitor_ZhiannBMS::store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells)
{
    for (uint8_t i = 0; i < ncells; i++) {
        const uint8_t idx = first_cell + i;
        if (idx < ARRAY_SIZE(_interim_state.cell_voltages.cells)) {
            _interim_state.cell_voltages.cells[idx] = le16toh_ptr(&data[2 * i]);
        }
    }
    _cells_seen = true;
}

void AP_BattMonitor_ZhiannBMS::handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    switch (frame_type) {
    case ZHIANN_FRAME_PACK_VOLT:
        if (frame.dlc >= 4) {
            const uint64_t now_us = AP_HAL::micros64();

            // duplicate-node detection: two packs interleaved on one node
            // deliver this ~500ms frame at sub-300ms spacing
            const uint32_t now_ms = AP_HAL::millis();
            if (_last_volt_ms != 0) {
                const uint32_t dt_ms = now_ms - _last_volt_ms;
                if (dt_ms < 300) {
                    _dup_score = MIN(_dup_score + 2, 20);
                } else if (dt_ms >= 400 && _dup_score > 0) {
                    _dup_score--;
                }
            }
            _last_volt_ms = now_ms;

            _interim_state.voltage = le16toh_ptr(&frame.data[2]) * 0.01f;
            if (frame.dlc >= 8) {
                // BMS sign convention is discharge-negative; ArduPilot's
                // is discharge-positive
                const int32_t raw = (int32_t)le32toh_ptr(&frame.data[4]);
                _interim_state.current_amps =
                    -raw * ZHIANN_CURRENT_SCALE * _curr_mult;
                // update_consumed skips the first frame
                // (last_time_micros == 0) and gaps over 2s
                update_consumed(_interim_state,
                                uint32_t(now_us - _last_frame_us));
                _current_seen = true;
            }
            _interim_state.last_time_micros = uint32_t(now_us);
            _last_frame_us = now_us;
        }
        break;

    case ZHIANN_FRAME_TEMP_SOC:
        if (frame.dlc >= 6) {
            // two sensors, signed 0.1C; report the hotter one
            const float t1 = (int16_t)le16toh_ptr(&frame.data[0]) * 0.1f;
            const float t2 = (int16_t)le16toh_ptr(&frame.data[2]) * 0.1f;
            _interim_state.temperature = MAX(t1, t2);
            _interim_state.temperature_time = AP_HAL::millis();
            _soc_pct = MIN(le16toh_ptr(&frame.data[4]) / 10, 100);
            _soc_valid = true;
            _fine_soc_ms = AP_HAL::millis();
        }
        break;

    case ZHIANN_FRAME_SOC_COARSE:
        // coarse 1% SOC: used only while the fine 0.1% frame is absent
        if (frame.dlc >= 2 &&
            (_fine_soc_ms == 0 ||
             AP_HAL::millis() - _fine_soc_ms > ZHIANN_FINE_SOC_STALE_MS)) {
            _soc_pct = MIN(le16toh_ptr(&frame.data[0]), 100);
            _soc_valid = true;
        }
        break;

    case ZHIANN_FRAME_ALARM:
        _alarm_bits = le16toh_ptr(&frame.data[0]);
        _warning_bits = frame.dlc >= 4 ? le16toh_ptr(&frame.data[2]) : 0;
        _alarm_ms = AP_HAL::millis();
        break;

    default:
        for (const auto &m : zhiann_cell_map) {
            if (m.frame_type == frame_type) {
                if (frame.dlc >= m.offset + 2 * m.ncells) {
                    store_cells(m.first_cell, &frame.data[m.offset], m.ncells);
                }
                break;
            }
        }
        break;
    }
}

void AP_BattMonitor_ZhiannBMS::read()
{
    WITH_SEMAPHORE(_sem);

    // 64-bit micros: no wraparound resurrection after long silence
    const uint64_t tnow_us = AP_HAL::micros64();
    if (_last_frame_us == 0 ||
        (tnow_us - _last_frame_us) > ZHIANN_TIMEOUT_US) {
        // BMS gone: report zero rather than freezing the last reading.
        // 0V is ArduPilot's "no reading" convention and cannot trigger
        // the voltage failsafes, which are guarded by voltage > 0
        _state.healthy = false;
        _state.voltage = 0;
        _state.current_amps = 0;
        _soc_valid_pub = false;
        _has_cell_voltages = false;
        _has_temperature = false;
        return;
    }

    _state.healthy = true;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells,
           sizeof(_state.cell_voltages.cells));

    _soc_pct_pub = _soc_pct;
    _soc_valid_pub = _soc_valid;
    _has_current = _current_seen;
    _has_cell_voltages = _cells_seen;
    _has_temperature = (_interim_state.temperature_time != 0) &&
        ((AP_HAL::millis() - _interim_state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT);

    // two packs sharing this node interleave their data: readings are a
    // physically meaningless mixture, so report unhealthy and tell the
    // operator why (with hysteresis so a single glitch does not flap)
    if (!_dup_active && _dup_score >= 10) {
        _dup_active = true;
    } else if (_dup_active && _dup_score <= 3) {
        _dup_active = false;
    }
    if (_dup_active) {
        _state.healthy = false;
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _dup_warn_ms > 30000) {
            _dup_warn_ms = now_ms;
            const int32_t serial = _params._serial_number.get();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: duplicate pack on node %d",
                          int(serial >= 0 ? serial : _auto_node));
        }
    }

    // publish faults from the BMS alarm frame; alarm state expires with
    // the standard battery timeout once the frames stop
    uint32_t faults = 0;
    if (_alarm_ms != 0 &&
        (AP_HAL::millis() - _alarm_ms) <= AP_BATT_MONITOR_TIMEOUT) {
        const uint16_t a = _alarm_bits;
        if (a & (ZHIANN_ALARM_DIS_OVERCURRENT | ZHIANN_ALARM_CHG_OVERCURRENT)) {
            faults |= MAV_BATTERY_FAULT_OVER_CURRENT;
        }
        if (a & ZHIANN_ALARM_HIGH_TEMP) {
            faults |= MAV_BATTERY_FAULT_OVER_TEMPERATURE;
        }
        if (a & ZHIANN_ALARM_LOW_TEMP) {
            faults |= MAV_BATTERY_FAULT_UNDER_TEMPERATURE;
        }
        if (a & ZHIANN_ALARM_CELL_UNDERVOLT) {
            faults |= MAV_BATTERY_FAULT_DEEP_DISCHARGE;
        }
        if (a & ZHIANN_ALARM_CELL_OVERVOLT) {
            faults |= MAV_BATTERY_FAULT_SPIKES;
        }
        if (a & (ZHIANN_ALARM_SEVERE | ZHIANN_ALARM_TEMP_SENSOR)) {
            faults |= MAV_BATTERY_FAULT_CELL_FAIL;
        }
        // a damaged battery or dead AFE must not pass arming
        if (a & ZHIANN_ALARM_SEVERE) {
            _state.healthy = false;
        }
    }
    _fault_bitmask = faults;
}

bool AP_BattMonitor_ZhiannBMS::capacity_remaining_pct(uint8_t &percentage) const
{
    if (_state.healthy && _soc_valid_pub) {
        percentage = _soc_pct_pub;
        return true;
    }
    // no BMS SOC: fall back to the coulomb count from the integrated
    // current and BATTn_CAPACITY
    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
