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
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include <string.h>

// frame classification and value decoding live in the shared pure
// header so unit tests can replay captured frames against them
#include "AP_BattMonitor_ZhiannBMS_decode.h"

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
uint16_t AP_BattMonitor_ZhiannBMS::_nodes_announced;
uint32_t AP_BattMonitor_ZhiannBMS::_unmapped_warn_ms;

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
    memset(_cells24, 0xFF, sizeof(_cells24));

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

    ZhiannBMS::Classified cls;
    if (!ZhiannBMS::classify(id, cls)) {
        return false;
    }

    // BMS alarm frame (PF 0x24, spec 6.8): never observed on the healthy
    // bench bus, listened for opportunistically. SA carries the pack node
    // in the broadcast profile; an unattributable alarm goes to every
    // instance - over-reporting a battery fault beats missing one
    if (cls.type == ZhiannBMS::FRAME_ALARM) {
        if (frame.dlc < 4) {
            return false;
        }
        bool attributed = false;
        for (uint8_t i = 0; i < _num_instances; i++) {
            if (_instances[i]->matches_node(cls.node)) {
                _instances[i]->handle_pack_frame(ZhiannBMS::FRAME_ALARM, frame);
                attributed = true;
            }
        }
        if (!attributed) {
            for (uint8_t i = 0; i < _num_instances; i++) {
                _instances[i]->handle_pack_frame(ZhiannBMS::FRAME_ALARM, frame);
            }
        }
        return true;
    }

    const uint8_t node = cls.node;
    const uint8_t frame_type = cls.type;

    // fleet inventory: announce each node once so the operator can see
    // the pack-to-node map (and notice when a claim has migrated)
    if (!(_nodes_announced & (1U << node))) {
        _nodes_announced |= (1U << node);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ZhiannBMS: pack on node %u", node);
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

    // a pack is broadcasting on a node no battery instance is mapped to:
    // most likely its node claim migrated (observed when bus membership
    // changes). Tell the operator which node to look at
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _unmapped_warn_ms > 30000) {
        _unmapped_warn_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ZhiannBMS: pack on node %u not mapped to any battery",
                      node);
    }
    return false;
}

void AP_BattMonitor_ZhiannBMS::store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells)
{
    for (uint8_t i = 0; i < ncells; i++) {
        const uint8_t idx = first_cell + i;
        const uint16_t mv = le16toh_ptr(&data[2 * i]);
        if (idx < ARRAY_SIZE(_interim_state.cell_voltages.cells)) {
            _interim_state.cell_voltages.cells[idx] = mv;
        }
        if (idx < ARRAY_SIZE(_cells24)) {
            _cells24[idx] = mv;
        }
    }
    _cells_seen = true;
}

void AP_BattMonitor_ZhiannBMS::handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    switch (frame_type) {
    case ZhiannBMS::FRAME_PACK_VOLT:
        if (frame.dlc >= 4) {
            const uint64_t now_us = AP_HAL::micros64();

            // duplicate-node detection: two packs interleaved on one node
            // deliver this ~500ms frame at sub-300ms spacing
            _dup.feed(AP_HAL::millis());

            _interim_state.voltage = ZhiannBMS::pack_voltage(frame.data);
            if (frame.dlc >= 8) {
                _interim_state.current_amps =
                    ZhiannBMS::current_amps(frame.data) * _curr_mult;
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

    case ZhiannBMS::FRAME_TEMP_SOC:
        _last_soc_ms = AP_HAL::millis();
        if (frame.dlc >= 6) {
            // two sensors, signed 0.1C; report the hotter one
            _temp1_c = ZhiannBMS::temp1_c(frame.data);
            _temp2_c = ZhiannBMS::temp2_c(frame.data);
            _interim_state.temperature = MAX(_temp1_c, _temp2_c);
            _interim_state.temperature_time = AP_HAL::millis();
            _soc_pct = ZhiannBMS::soc_fine_pct(frame.data);
            _soc_valid = true;
            _fine_soc_ms = AP_HAL::millis();
        }
        break;

    case ZhiannBMS::FRAME_SOC_COARSE:
        // this frame keeps flowing in pack standby, when the detail
        // frames stop: it distinguishes standby from gone-from-bus
        _last_soc_ms = AP_HAL::millis();
        if (frame.dlc >= 4) {
            _soc_frame_vmir = ZhiannBMS::soc_voltage_mirror(frame.data);
        }
        // coarse 1% SOC: used only while the fine 0.1% frame is absent
        if (frame.dlc >= 2 &&
            (_fine_soc_ms == 0 ||
             AP_HAL::millis() - _fine_soc_ms > ZHIANN_FINE_SOC_STALE_MS)) {
            _soc_pct = ZhiannBMS::soc_coarse_pct(frame.data);
            _soc_valid = true;
        }
        break;

    case ZhiannBMS::FRAME_ALARM:
        _alarm_bits = le16toh_ptr(&frame.data[0]);
        _warning_bits = frame.dlc >= 4 ? le16toh_ptr(&frame.data[2]) : 0;
        _alarm_ms = AP_HAL::millis();
        break;

    default:
        for (const auto &m : ZhiannBMS::CELL_MAP) {
            if (m.frame_type == frame_type) {
                if (frame.dlc >= m.offset + 2 * m.ncells) {
                    store_cells(m.first_cell, &frame.data[m.offset], m.ncells);
                }
                if (m.frame_type == ZhiannBMS::FRAME_CELL_1_2 && frame.dlc >= 4) {
                    _cell_count = frame.data[2];
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

        // SOC frames still arriving means the pack is present but in
        // standby (its detail frames stop): tell the operator, since the
        // fix is pressing the pack's power button, not checking wiring
        const uint32_t now_ms = AP_HAL::millis();
        _standby = _last_soc_ms != 0 &&
            (now_ms - _last_soc_ms) <= AP_BATT_MONITOR_TIMEOUT;
        if (_standby && (now_ms - _standby_warn_ms) > 30000) {
            _standby_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: pack on node %d in standby",
                          int(bound_node()));
        }
        log_zbms();
        return;
    }
    _standby = false;

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
    // operator why (hysteresis inside the detector stops flapping)
    _dup_active = _dup.active();
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

    log_zbms();
}

// three dataflash messages per instance at 2Hz:
//   ZBMS: node, state flags, pack voltage/current, fine SOC, both temps,
//         alarm/warning words, SOC-frame voltage mirror, cell count, age
//   ZBC1: cells 1-12 (mV)   ZBC2: cells 13-24 (mV)
void AP_BattMonitor_ZhiannBMS::log_zbms()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (_last_frame_us == 0 || now_ms - _last_log_ms < 500) {
        return;
    }
    _last_log_ms = now_ms;

    const bool soc_fine = _fine_soc_ms != 0 &&
        (now_ms - _fine_soc_ms) <= ZHIANN_FINE_SOC_STALE_MS;
    const uint8_t flags = (_state.healthy ? 1U : 0) |
                          (_standby ? 2U : 0) |
                          (_dup_active ? 4U : 0) |
                          (soc_fine ? 8U : 0) |
                          (_current_seen ? 16U : 0);
    const uint64_t now_us = AP_HAL::micros64();
    const uint16_t age_ms = MIN(uint32_t((now_us - _last_frame_us) / 1000ULL),
                                (uint32_t)UINT16_MAX);

    AP::logger().WriteStreaming(
        "ZBMS", "TimeUS,Inst,Node,Flag,Volt,Curr,SOC,T1,T2,Alrm,Warn,Vmir,NCel,Age",
        "s#--vA%OO-----", "F---00000-----",
        "QBbBffBffHHHBH",
        now_us,
        _state.instance,
        (int8_t)bound_node(),
        flags,
        (double)_interim_state.voltage,
        (double)_interim_state.current_amps,
        _soc_pct,
        (double)_temp1_c,
        (double)_temp2_c,
        _alarm_bits,
        _warning_bits,
        _soc_frame_vmir,
        _cell_count,
        age_ms);

    const uint16_t *c = _cells24;
    AP::logger().WriteStreaming(
        "ZBC1", "TimeUS,Inst,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12",
        "s#vvvvvvvvvvvv", "F-CCCCCCCCCCCC",
        "QBHHHHHHHHHHHH",
        now_us, _state.instance,
        c[0], c[1], c[2], c[3], c[4], c[5],
        c[6], c[7], c[8], c[9], c[10], c[11]);
    AP::logger().WriteStreaming(
        "ZBC2", "TimeUS,Inst,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23,V24",
        "s#vvvvvvvvvvvv", "F-CCCCCCCCCCCC",
        "QBHHHHHHHHHHHH",
        now_us, _state.instance,
        c[12], c[13], c[14], c[15], c[16], c[17],
        c[18], c[19], c[20], c[21], c[22], c[23]);
#endif
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
