/*
  battery monitor for the Zhiann CAN BMS

  protocol reverse engineered from live packs (2026-07): 1 Mbit/s classic
  CAN, all IDs 29-bit extended, values little-endian u16 unless noted.

  Multiple packs share one bus. Pack n transmits the ID block
  0x2E0941 + 0x20*n plus an SOC frame at 0x401A100 + n (4 packs observed
  live; the ID arithmetic is extrapolated up to node 15, the SOC frame's
  node nibble - ArduPilot itself supports 9 monitors). Node numbering
  is a firmware claim that can migrate or collide. Frame types within a
  pack block (offset
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
   - one bus only: CANSensor supports one CAN interface. Configure this
     protocol on exactly one CAN port.
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

extern const AP_HAL::HAL& hal;

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
// alarms reported to the GCS as MAV_BATTERY_FAULT_CELL_FAIL
#define ZHIANN_ALARM_SEVERE  (ZHIANN_ALARM_BATT_DAMAGED | ZHIANN_ALARM_AFE_FAULT)

// operator-facing names for the alarm word bits
static const struct {
    uint16_t mask;
    const char *name;
} zhiann_alarm_names[] = {
    { ZHIANN_ALARM_CHG_OVERCURRENT, "chg OC" },
    { ZHIANN_ALARM_TEMP_SENSOR,     "temp sensor" },
    { ZHIANN_ALARM_CELL_OVERVOLT,   "cell OV" },
    { ZHIANN_ALARM_CELL_UNDERVOLT,  "cell UV" },
    { ZHIANN_ALARM_HIGH_TEMP,       "high temp" },
    { ZHIANN_ALARM_LOW_TEMP,        "low temp" },
    { ZHIANN_ALARM_AFE_FAULT,       "AFE fault" },
    { ZHIANN_ALARM_BATT_DAMAGED,    "batt damaged" },
    { ZHIANN_ALARM_DIS_OVERCURRENT, "dis OC" },
};

// consider the BMS gone after the codebase-wide battery timeout
#define ZHIANN_TIMEOUT_US      (AP_BATT_MONITOR_TIMEOUT * 1000ULL)

// accept the coarse 1% SOC only when the fine 0.1% frame (nominally
// every 500ms) has been silent this long
#define ZHIANN_FINE_SOC_STALE_MS  2500

// Clean captures stayed below this full-cell-sum mismatch; every mismatch
// above 1V in the corpus came from a known duplicate-node collision.
#define ZHIANN_CELL_SUM_TOLERANCE_V  1.0f

// Across 35,613 complete captured bursts, all seven slices span 22..55ms.
// This margin tolerates bus/scheduler jitter but rejects a set assembled
// from different nominal 500ms generations after a slice is lost.
#define ZHIANN_CELL_SNAPSHOT_SPAN_MS  250

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

// Use a dedicated CANSensor instead of MultiCAN. MultiCAN's callback list is
// global across protocols, which can leak matching traffic between physical
// buses and lets wildcard consumers starve this safety-critical backend.
class AP_BattMonitor_ZhiannBMS_CAN : public CANSensor {
public:
    explicit AP_BattMonitor_ZhiannBMS_CAN(AP_BattMonitor_ZhiannBMS &owner) :
        CANSensor("ZhiannBMS"),
        _owner(owner)
    {
        register_driver(AP_CAN::Protocol::ZhiannBMS);
    }

    void handle_frame(AP_HAL::CANFrame &frame) override
    {
        _owner.dispatch_frame(frame);
    }

private:
    AP_BattMonitor_ZhiannBMS &_owner;
};

AP_BattMonitor_ZhiannBMS_CAN *AP_BattMonitor_ZhiannBMS::_can_driver;
AP_BattMonitor_ZhiannBMS *AP_BattMonitor_ZhiannBMS::_instances[AP_BATT_MONITOR_MAX_INSTANCES];
uint8_t AP_BattMonitor_ZhiannBMS::_num_instances;
uint16_t AP_BattMonitor_ZhiannBMS::_nodes_announced;
uint32_t AP_BattMonitor_ZhiannBMS::_unmapped_warn_ms[ZhiannBMS::MAX_NODE + 1];
bool AP_BattMonitor_ZhiannBMS::_misconfig_checked;

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
    if (_can_driver == nullptr) {
        _can_driver = NEW_NOTHROW AP_BattMonitor_ZhiannBMS_CAN(*this);
        if (_can_driver == nullptr) {
            AP_BoardConfig::allocation_error("ZhiannBMS CAN driver");
        }
    }
}

void AP_BattMonitor_ZhiannBMS::init()
{
    const int32_t serial = _params._serial_number.get();
    if (serial == -1) {
        _configured_node = -1;
    } else if (serial >= 0 && serial <= ZhiannBMS::MAX_NODE) {
        _configured_node = int8_t(serial);
    } else {
        _configured_node = -2;
    }
}

bool AP_BattMonitor_ZhiannBMS::matches_node(uint8_t node) const
{
    if (_configured_node >= 0) {
        return node == uint8_t(_configured_node);
    }
    return _configured_node == -1 && _auto_node >= 0 &&
           node == uint8_t(_auto_node);
}

bool AP_BattMonitor_ZhiannBMS::node_claimed(uint8_t node)
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        if (_instances[i]->_configured_node == int8_t(node)) {
            return true;
        }
    }
    return false;
}

// dedicated CAN-driver callback, dispatching across all battery instances
bool AP_BattMonitor_ZhiannBMS::dispatch_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() ||
        frame.isErrorFrame() || frame.isCanFDFrame()) {
        return false;
    }
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    ZhiannBMS::Classified cls;
    if (!ZhiannBMS::classify(id, cls)) {
        return false;
    }
    if (!ZhiannBMS::valid_dlc(cls.type, frame.dlc)) {
        return false;
    }

    // BMS alarm frame (PF 0x24, spec 6.8): never observed on the healthy
    // bench bus. Its SA is a standard protocol address, not the proprietary
    // broadcast node, and unassigned packs all use SA=0. Without a verified
    // identity mapping, conservatively deliver every alarm to every pack.
    if (cls.type == ZhiannBMS::FRAME_ALARM) {
        for (uint8_t i = 0; i < _num_instances; i++) {
            _instances[i]->handle_pack_frame(ZhiannBMS::FRAME_ALARM, frame);
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
            if (inst._configured_node == -1 && inst._auto_node < 0) {
                {
                    WITH_SEMAPHORE(inst._sem);
                    inst._auto_node = int8_t(node);
                }
                inst.handle_pack_frame(frame_type, frame);
                return true;
            }
        }
    }

    // a pack is broadcasting on a node no battery instance is mapped to:
    // most likely its node claim migrated (observed when bus membership
    // changes). Tell the operator which node to look at
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < _num_instances; i++) {
        _instances[i]->note_unmapped(now_ms);
    }
    if (_unmapped_warn_ms[node] == 0 ||
        now_ms - _unmapped_warn_ms[node] >= 30000) {
        _unmapped_warn_ms[node] = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ZhiannBMS: pack on node %u not mapped to any battery",
                      node);
    }
    return false;
}

void AP_BattMonitor_ZhiannBMS::note_unmapped(uint32_t now_ms)
{
    WITH_SEMAPHORE(_sem);
    _unmapped_ms = now_ms;
}

void AP_BattMonitor_ZhiannBMS::reconcile_consumption(uint16_t soc_tenths)
{
    // PACK_VOLT normally starts the 500ms burst. Waiting for it avoids
    // reconciling consumed Wh from a zero or stale voltage when SOC arrives first.
    if (!_current_seen ||
        !ZhiannBMS::pack_voltage_valid(_interim_state.voltage)) {
        return;
    }
    const float previous_mah = _interim_state.consumed_mah;
    const float reconciled_mah = ZhiannBMS::reconciled_consumed_mah(
        previous_mah, _params._pack_capacity, soc_tenths);
    if (reconciled_mah > previous_mah) {
        // The BMS has a measured low-current deadband. A falling SOC must
        // therefore be allowed to raise the capacity-failsafe floor, while an
        // SOC increase/noise must never discard current-integrated usage.
        // This same floor governs (re-)seeding: consumption never moves down,
        // even on the first SOC after a session reset.
        const float delta_mah = reconciled_mah - previous_mah;
        _interim_state.consumed_mah = reconciled_mah;
        _interim_state.consumed_wh += delta_mah *
            0.001f * _interim_state.voltage;
    }
}

void AP_BattMonitor_ZhiannBMS::handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    switch (frame_type) {
    case ZhiannBMS::FRAME_PACK_VOLT:
        {
            const uint64_t now_us = AP_HAL::micros64();

            // voltage anchors liveness: an implausible voltage rejects the
            // whole frame. Current is judged independently below, so a
            // current-register fault cannot silence an otherwise live pack.
            const float voltage = ZhiannBMS::pack_voltage(frame.data);
            if (!ZhiannBMS::pack_voltage_valid(voltage)) {
                break;
            }
            const float current = ZhiannBMS::current_amps(frame.data) * _curr_mult;

            // duplicate-node detection: two packs interleaved on one node
            // deliver this ~500ms frame at sub-300ms spacing
            _dup.feed(AP_HAL::millis());
            _interim_state.voltage = voltage;
            if (ZhiannBMS::current_valid(current)) {
                _interim_state.current_amps = current;
                // update_consumed skips the first frame and gaps over 2s
                update_consumed(_interim_state,
                                ZhiannBMS::consumption_dt_us(now_us - _last_frame_us));
                _current_seen = true;
                _current_fault = false;
            } else {
                // keep voltage/liveness but hold has_current down and skip
                // consumption integration until a plausible reading returns
                _current_fault = true;
                const uint32_t now_ms = AP_HAL::millis();
                if (_current_warn_ms == 0 ||
                    now_ms - _current_warn_ms >= 30000) {
                    _current_warn_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "ZhiannBMS: implausible current from pack on node %d",
                                  int(bound_node()));
                }
            }
            _interim_state.last_time_micros = uint32_t(now_us);
            _last_frame_us = now_us;
        }
        break;

    case ZhiannBMS::FRAME_TEMP_SOC:
        _last_soc_ms = AP_HAL::millis();
        // two sensors, signed 0.1C; report the hotter plausible one. One
        // failed sensor must not blank pack temperature entirely
        {
            const float temp1 = ZhiannBMS::temp1_c(frame.data);
            const float temp2 = ZhiannBMS::temp2_c(frame.data);
            // raw values are logged as-is so a faulty sensor is identifiable
            _temp1_c = temp1;
            _temp2_c = temp2;
            float selected;
            if (ZhiannBMS::select_temperature(temp1, temp2, selected)) {
                _interim_state.temperature = selected;
                _interim_state.temperature_time = _last_soc_ms;
            } else if (_temp_warn_ms == 0 ||
                       _last_soc_ms - _temp_warn_ms >= 30000) {
                _temp_warn_ms = _last_soc_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "ZhiannBMS: temperature sensor fault on node %d",
                              int(bound_node()));
            }
        }
        if (ZhiannBMS::soc_fine_valid(frame.data)) {
            const uint16_t soc_tenths = ZhiannBMS::soc_fine_tenths(frame.data);
            _soc_pct = soc_tenths / 10;
            _soc_valid = true;
            _soc_valid_ms = _last_soc_ms;
            _fine_soc_ms = _last_soc_ms;
            reconcile_consumption(soc_tenths);
        }
        break;

    case ZhiannBMS::FRAME_SOC_COARSE:
        // this frame keeps flowing in pack standby, when the detail
        // frames stop: it distinguishes standby from gone-from-bus
        _last_soc_ms = AP_HAL::millis();
        _soc_frame_vmir = ZhiannBMS::soc_voltage_mirror(frame.data);
        // compare only against a recent (<=500ms) pack-voltage sample: an
        // older one may straddle a load step and fake a mismatch
        if (_last_frame_us != 0 &&
            (AP_HAL::micros64() - _last_frame_us) <= 500000ULL) {
            _coherence_detector.feed_vmir(ZhiannBMS::pack_vmir_coherent(
                _interim_state.voltage, _soc_frame_vmir,
                ZHIANN_CELL_SUM_TOLERANCE_V));
        }
        // coarse 1% SOC: used only while the fine 0.1% frame is absent
        if (ZhiannBMS::soc_coarse_valid(frame.data) &&
            (_fine_soc_ms == 0 ||
             AP_HAL::millis() - _fine_soc_ms > ZHIANN_FINE_SOC_STALE_MS)) {
            _soc_pct = ZhiannBMS::soc_coarse_pct(frame.data);
            _soc_valid = true;
            _soc_valid_ms = _last_soc_ms;
            reconcile_consumption(uint16_t(_soc_pct) * 10U);
        }
        break;

    case ZhiannBMS::FRAME_ALARM: {
        const uint32_t now_ms = AP_HAL::millis();
        const uint16_t alarm_bits = le16toh_ptr(&frame.data[0]);
        const uint16_t warning_bits = le16toh_ptr(&frame.data[2]);
        if (_num_instances == 1) {
            _alarm_bits = alarm_bits;
            _warning_bits = warning_bits;
            _alarm_ms = now_ms;
            _warning_ms = now_ms;
        } else {
            // Alarm source addresses have no proven mapping to proprietary
            // pack nodes. Preserve the union so a clear frame from another
            // pack cannot erase an active fault; non-severe MAV faults expire
            // after the last nonzero alarm rather than on anonymous zeroes.
            if (alarm_bits != 0) {
                if (!ZhiannBMS::fresh_ms(now_ms, _alarm_ms,
                                         AP_BATT_MONITOR_TIMEOUT)) {
                    _alarm_bits = 0;
                }
                _alarm_bits |= alarm_bits;
                _alarm_ms = now_ms;
            }
            if (warning_bits != 0) {
                if (!ZhiannBMS::fresh_ms(now_ms, _warning_ms,
                                         AP_BATT_MONITOR_TIMEOUT)) {
                    _warning_bits = 0;
                }
                _warning_bits |= warning_bits;
                _warning_ms = now_ms;
            }
        }
        break;
    }

    default:
        if (frame_type == ZhiannBMS::FRAME_CELL_1_2) {
            // PACK_VOLT begins the same canonical burst. Preserve that
            // generation's voltage, plus its arrival time for the freshness
            // gate below, so readers never compare committed cells with a
            // later generation's voltage during a load step.
            _cell_accumulator_voltage = _interim_state.voltage;
            _cell_voltage_sample_us = _last_frame_us;
        }
        if (frame_type == ZhiannBMS::FRAME_CELL_1_2) {
            // the cell-count word rides in this frame; a pack reporting a
            // different chemistry/series count deserves a loud explanation
            // for why cell voltages never publish
            const uint16_t reported = ZhiannBMS::u16(&frame.data[2]);
            if (reported != 24) {
                const uint32_t now_ms = AP_HAL::millis();
                if (_cellcount_warn_ms == 0 ||
                    now_ms - _cellcount_warn_ms >= 30000) {
                    _cellcount_warn_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "ZhiannBMS: pack on node %d reports %u cells, expected 24",
                                  int(bound_node()), unsigned(reported));
                }
            }
        }
        if (_cell_accumulator.feed(frame_type, frame.data, AP_HAL::millis(),
                                   ZHIANN_CELL_SNAPSHOT_SPAN_MS)) {
            const uint16_t *cells = _cell_accumulator.cells();
            // compare only against a recent (<=500ms) voltage sample from
            // this burst: an older one may straddle a load step and fake a
            // mismatch (same freshness gate as the SOC voltage mirror)
            bool coherent = false;
            if (_cell_voltage_sample_us != 0 &&
                (AP_HAL::micros64() - _cell_voltage_sample_us) <= 500000ULL) {
                coherent = ZhiannBMS::pack_cells_coherent(
                    _cell_accumulator_voltage, cells, 24,
                    ZHIANN_CELL_SUM_TOLERANCE_V);
                _coherence_detector.feed_cellsum(coherent);
            }
            // an incoherent generation is detector evidence only: it must
            // never reach the published cell state. Publication stays on
            // the previous coherent snapshot until that ages past the 5s
            // freshness window in read()
            if (coherent) {
                for (uint8_t i = 0; i < ARRAY_SIZE(_cells24); i++) {
                    _cells24[i] = cells[i];
                    if (i < ARRAY_SIZE(_interim_state.cell_voltages.cells)) {
                        _interim_state.cell_voltages.cells[i] = cells[i];
                    }
                }
                _cell_count = uint8_t(_cell_accumulator.cell_count());
                _last_coherent_snapshot_ms = AP_HAL::millis();
            }
            _cell_accumulator.reset();
        }
        break;
    }
}

// one-time audit of static configuration errors, run from the first read()
// (all instances exist and have snapshotted their serial parameter by then)
void AP_BattMonitor_ZhiannBMS::announce_misconfiguration()
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        // parameter-name suffix digit: BATT for instance 0, BATT2.. beyond
        // (same convention as the front-end's param prefix)
        char pfx_i[3] {};
        if (_instances[i]->_state.instance > 0) {
            hal.util->snprintf(pfx_i, sizeof(pfx_i), "%X",
                               unsigned(_instances[i]->_state.instance + 1));
        }
        const int8_t node = _instances[i]->_configured_node;
        if (node == -2) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: BATT%s_SERIAL_NUM invalid", pfx_i);
            continue;
        }
        if (node < 0) {
            continue;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (_instances[j]->_configured_node == node) {
                char pfx_j[3] {};
                if (_instances[j]->_state.instance > 0) {
                    hal.util->snprintf(pfx_j, sizeof(pfx_j), "%X",
                                       unsigned(_instances[j]->_state.instance + 1));
                }
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "ZhiannBMS: BATT%s_SERIAL_NUM duplicates BATT%s",
                              pfx_i, pfx_j);
                break;
            }
        }
    }

    // CANSensor binds only the first port carrying this protocol
    uint8_t ports = 0;
    for (uint8_t i = 0; i < HAL_MAX_CAN_PROTOCOL_DRIVERS; i++) {
        if (AP::can().get_driver_type(i) == AP_CAN::Protocol::ZhiannBMS) {
            ports++;
        }
    }
    if (ports > 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ZhiannBMS: protocol on multiple CAN ports; only first is used");
    }
}

void AP_BattMonitor_ZhiannBMS::read()
{
    if (!_misconfig_checked) {
        _misconfig_checked = true;
        announce_misconfiguration();
    }

    WITH_SEMAPHORE(_sem);

    // 64-bit micros: no wraparound resurrection after long silence
    const uint64_t tnow_us = AP_HAL::micros64();
    const uint32_t now_ms = AP_HAL::millis();

    // Expire fault telemetry even when detail traffic has stopped.
    if (_alarm_ms != 0 &&
        !ZhiannBMS::fresh_ms(now_ms, _alarm_ms, AP_BATT_MONITOR_TIMEOUT)) {
        _alarm_bits = 0;
        _alarm_ms = 0;
    }
    if (_warning_ms != 0 &&
        !ZhiannBMS::fresh_ms(now_ms, _warning_ms, AP_BATT_MONITOR_TIMEOUT)) {
        _warning_bits = 0;
        _warning_ms = 0;
    }
    // Alarms are messages only: they feed the MAVLink fault bitmask, the
    // ZBMS log and a periodic operator warning, and never gate health.
    uint32_t faults = 0;
    if (ZhiannBMS::fresh_ms(now_ms, _alarm_ms, AP_BATT_MONITOR_TIMEOUT)) {
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
        // remind the operator every 10s while any alarm bit stays active,
        // naming the bits. Alarms are anonymous unions delivered to every
        // instance, so only the first instance speaks for the fleet
        if (a != 0 && this == _instances[0] &&
            (_alarm_gcs_ms == 0 || now_ms - _alarm_gcs_ms >= 10000)) {
            _alarm_gcs_ms = now_ms;
            char names[40] {};
            uint8_t used = 0;
            uint16_t unnamed = a;
            for (const auto &an : zhiann_alarm_names) {
                if (!(a & an.mask)) {
                    continue;
                }
                unnamed &= ~an.mask;
                const int w = hal.util->snprintf(&names[used],
                                                 sizeof(names) - used, "%s%s",
                                                 used ? ", " : "", an.name);
                if (w <= 0 || uint32_t(used) + uint32_t(w) >= sizeof(names)) {
                    used = sizeof(names) - 1;
                    break;
                }
                used += uint8_t(w);
            }
            if (unnamed != 0 && used < sizeof(names) - 1) {
                hal.util->snprintf(&names[used], sizeof(names) - used,
                                   "%s0x%X", used ? ", " : "",
                                   unsigned(unnamed));
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ZhiannBMS: BMS alarm: %s",
                          names);
        }
    }
    _fault_bitmask = faults;
    _unmapped_active = ZhiannBMS::fresh_ms(now_ms, _unmapped_ms,
                                           AP_BATT_MONITOR_TIMEOUT);

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
        _coherence_fault = false;
        _dup_active = false;
        _dup.reset_qualification();
        _soc_valid = false;
        _soc_valid_ms = 0;
        _fine_soc_ms = 0;
        _interim_state.temperature_time = 0;
        _interim_state.last_time_micros = 0;
        // _current_seen and _has_current are deliberately preserved at
        // their last values, so when current was good at outage onset the
        // capacity failsafes remain armed across a BMS outage; consumption
        // re-seeding is floored by reconcile_consumption() when the pack
        // returns
        memset(_interim_state.cell_voltages.cells, 0xFF,
               sizeof(_interim_state.cell_voltages.cells));
        memset(_state.cell_voltages.cells, 0xFF,
               sizeof(_state.cell_voltages.cells));
        memset(_cells24, 0xFF, sizeof(_cells24));
        _cell_accumulator.reset();
        _cell_accumulator_voltage = 0;
        _cell_voltage_sample_us = 0;
        _last_coherent_snapshot_ms = 0;
        _cell_count = 0;
        // session boundary: a half-armed coherence strike from before the
        // outage must not pair with the new session's first mismatch (an
        // accumulated active score is deliberately preserved as evidence)
        _coherence_detector.clear_pending();

        // SOC frames still arriving means the pack is present but in
        // standby (its detail frames stop): tell the operator, since the
        // fix is pressing the pack's power button, not checking wiring
        _standby = ZhiannBMS::fresh_ms(now_ms, _last_soc_ms,
                                      AP_BATT_MONITOR_TIMEOUT);
        if (_standby && (_standby_warn_ms == 0 ||
                         now_ms - _standby_warn_ms >= 30000)) {
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
    // While the current register is faulted, publish 0A (the no-reading
    // convention) rather than the stale last-good reading. Note: with a
    // persistent current fault the mAh capacity failsafes are inactive
    // (has_current false) and the voltage failsafes are the backstop.
    _state.current_amps = _current_fault ? 0 : _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _soc_pct_pub = _soc_pct;
    _soc_valid_pub = _soc_valid &&
        ZhiannBMS::fresh_ms(now_ms, _soc_valid_ms, AP_BATT_MONITOR_TIMEOUT);
    _has_current = _current_seen && !_current_fault;
    _has_temperature = ZhiannBMS::fresh_ms(now_ms,
                                           _interim_state.temperature_time,
                                           AP_BATT_MONITOR_TIMEOUT);

    // An unmonitored pack on a parallel fleet is a system-level safety
    // violation even when this particular mapped pack is otherwise healthy.
    if (_unmapped_active) {
        _state.healthy = false;
    }

    // Publication and health key off the last COHERENT complete snapshot
    // staying inside the standard 5s freshness window: one incoherent
    // 500ms generation (e.g. a throttle punch straddling a burst) must not
    // blip health, while sustained incoherence still fails via both this
    // window and the coherence detector.
    const bool cell_snapshot_valid = _cell_count == 24 &&
        ZhiannBMS::fresh_ms(now_ms, _last_coherent_snapshot_ms,
                            AP_BATT_MONITOR_TIMEOUT);
    if (cell_snapshot_valid) {
        for (uint8_t i = 0; i < ARRAY_SIZE(_state.cell_voltages.cells); i++) {
            _state.cell_voltages.cells[i] =
                _interim_state.cell_voltages.cells[i];
        }
    } else {
        memset(_interim_state.cell_voltages.cells, 0xFF,
               sizeof(_interim_state.cell_voltages.cells));
        memset(_state.cell_voltages.cells, 0xFF,
               sizeof(_state.cell_voltages.cells));
        memset(_cells24, 0xFF, sizeof(_cells24));
        _cell_count = 0;
    }
    _coherence_fault = _coherence_detector.active();
    _has_cell_voltages = cell_snapshot_valid;
    if (_coherence_fault) {
        _state.healthy = false;
        if (_coherence_warn_ms == 0 || now_ms - _coherence_warn_ms >= 30000) {
            _coherence_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: incoherent data on node %d",
                          int(bound_node()));
        }
    }

    // A PACK frame alone is not a coherent battery snapshot. Do not permit
    // arming until SOC, temperature, cell count, and all seven cell slices
    // have arrived and remain fresh.
    if (!_soc_valid_pub || !_has_temperature || !_has_cell_voltages) {
        _state.healthy = false;
    }

    // two packs sharing this node interleave their data: readings are a
    // physically meaningless mixture, so report unhealthy and tell the
    // operator why (hysteresis inside the detector stops flapping)
    _dup_active = _dup.active();
    if (_dup_active || !_dup.qualified()) {
        _state.healthy = false;
    }
    if (_dup_active) {
        if (_dup_warn_ms == 0 || now_ms - _dup_warn_ms >= 30000) {
            _dup_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: duplicate pack on node %d",
                          int(bound_node()));
        }
    }

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
                          (_current_seen ? 16U : 0) |
                          (_unmapped_active ? 32U : 0) |
                          (_coherence_fault ? 64U : 0) |
                          (_current_fault ? 128U : 0);
    const uint64_t now_us = AP_HAL::micros64();
    const uint64_t age_ms_64 = (now_us - _last_frame_us) / 1000ULL;
    const uint16_t age_ms = age_ms_64 > UINT16_MAX ? UINT16_MAX :
                            uint16_t(age_ms_64);

    static_assert(sizeof("TimeUS,Inst,Node,Flag,Volt,Curr,SOC,T1,T2,Alm,Warn,Vmir,NCel,Age") <= LS_LABELS_SIZE,
                  "ZBMS labels exceed dataflash schema limit");
    // @LoggerMessage: ZBMS
    // @Description: Zhiann BMS pack status
    // @Field: TimeUS: Time since system startup
    // @Field: Inst: Battery monitor instance
    // @Field: Node: Proprietary broadcast node
    // @Field: Flag: Health, standby, collision, SOC source, current, unmapped and coherence flags
    // @Field: Volt: Pack voltage
    // @Field: Curr: Pack current, discharge positive
    // @Field: SOC: BMS state of charge
    // @Field: T1: Temperature sensor 1
    // @Field: T2: Temperature sensor 2
    // @Field: Alm: Vendor alarm bitmask
    // @Field: Warn: Vendor warning bitmask
    // @Field: Vmir: Raw SOC-frame voltage mirror, in 1/320 volt
    // @Field: NCel: Reported cell count
    // @Field: Age: Milliseconds since the last valid pack frame
    AP::logger().WriteStreaming(
        "ZBMS", "TimeUS,Inst,Node,Flag,Volt,Curr,SOC,T1,T2,Alm,Warn,Vmir,NCel,Age",
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
    // @LoggerMessage: ZBC1
    // @Description: Zhiann BMS cell voltages 1 through 12
    // @Field: TimeUS: Time since system startup
    // @Field: Inst: Battery monitor instance
    // @Field: V1: Cell 1 voltage
    // @Field: V2: Cell 2 voltage
    // @Field: V3: Cell 3 voltage
    // @Field: V4: Cell 4 voltage
    // @Field: V5: Cell 5 voltage
    // @Field: V6: Cell 6 voltage
    // @Field: V7: Cell 7 voltage
    // @Field: V8: Cell 8 voltage
    // @Field: V9: Cell 9 voltage
    // @Field: V10: Cell 10 voltage
    // @Field: V11: Cell 11 voltage
    // @Field: V12: Cell 12 voltage
    AP::logger().WriteStreaming(
        "ZBC1", "TimeUS,Inst,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12",
        "s#vvvvvvvvvvvv", "F-CCCCCCCCCCCC",
        "QBHHHHHHHHHHHH",
        now_us, _state.instance,
        c[0], c[1], c[2], c[3], c[4], c[5],
        c[6], c[7], c[8], c[9], c[10], c[11]);
    // @LoggerMessage: ZBC2
    // @Description: Zhiann BMS cell voltages 13 through 24
    // @Field: TimeUS: Time since system startup
    // @Field: Inst: Battery monitor instance
    // @Field: V13: Cell 13 voltage
    // @Field: V14: Cell 14 voltage
    // @Field: V15: Cell 15 voltage
    // @Field: V16: Cell 16 voltage
    // @Field: V17: Cell 17 voltage
    // @Field: V18: Cell 18 voltage
    // @Field: V19: Cell 19 voltage
    // @Field: V20: Cell 20 voltage
    // @Field: V21: Cell 21 voltage
    // @Field: V22: Cell 22 voltage
    // @Field: V23: Cell 23 voltage
    // @Field: V24: Cell 24 voltage
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
    // Preserve the base behavior for any future configuration that permits a
    // healthy current-only state. Current policy makes stale SOC unhealthy.
    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
