/*
  battery monitor for the Zhiann CAN BMS

  protocol reverse engineered from live packs (2026-07): 1 Mbit/s classic
  CAN, all IDs 29-bit extended, values little-endian u16 unless noted.

  Several packs share one bus, wired in parallel. Pack n transmits the ID
  block 0x2E0941 + 0x20*n plus an SOC frame at 0x401A100 + n. The node
  number is a claim the pack firmware makes for itself: it can migrate, and
  two packs can end up claiming the same one. Frame types within a pack
  block (offset from 0x...41):

    +0x00  cell voltages 19..22 (mV)
    +0x01  temperature1 (0.1 C), temperature2 (0.1 C), SOC (0.1 %)
    +0x02  unknown, cell count, cell voltages 1..2 (mV)
    +0x03  cell voltages 3..6 (mV)
    +0x04  cell voltages 7..10 (mV)
    +0x05  cell voltages 11..14 (mV)
    +0x06  cell voltages 15..18 (mV)
    +0x09  cell voltages 23..24 (mV)
    +0x10  counter/crc, pack voltage (10 mV), current (s32, 1 mA,
           negative = discharge, reads 0 below a few amps)

  0x401A100+n: SOC (%), pack voltage mirror (1/320 V), temperature?
  0x402A100+n: 2 s frame carrying a per-pack identity in bytes 4..7. Not
    every occupied node emits it and the rule is not known.

  The pack block repeats every ~500 ms. Current is 1 mA/LSB: the pack
  broadcasts its own 44.0 Ah rated capacity, and coulomb-counting flights
  against the packs' own SOC agrees with that scale. Use BATTn_CURR_MULT to
  trim in the field.

  THE PACK SET IS PUBLISHED AS ONE BATTERY. Every node found on the bus is
  consumed and reduced to a single reading: mean voltage, summed current,
  mean SOC, highest temperature, mean cell voltages. There is no node-to-
  instance mapping and therefore nothing for a node collision to break - a
  collision simply means one less node carrying two packs' frames, and the
  averages absorb it. The node count and any suspected collision are
  reported to the operator as information, never as a health failure.

  Configure exactly one battery monitor with this type.

  Limitations:
   - one bus only: CANSensor supports one CAN interface. Configure this
     protocol on exactly one CAN port.
 */
#include "AP_BattMonitor_ZhiannBMS.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
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

// a pack is part of the set while its frames are this fresh
#define ZHIANN_NODE_TIMEOUT_MS  AP_BATT_MONITOR_TIMEOUT

// accept the coarse 1% SOC only when the fine 0.1% frame (nominally
// every 500ms) has been silent this long
#define ZHIANN_FINE_SOC_STALE_MS  2500

// Across 35,613 complete captured bursts, all seven slices span 22..55ms.
#define ZHIANN_CELL_SNAPSHOT_SPAN_MS  250

// Packs do not power on together: four packs switched on by one operator
// action appeared over 2.5s on the bench. Wait for the set to stop growing
// before announcing how many were found.
#define ZHIANN_BUS_SETTLE_MS  5000

// Current is summed and integrated, so a stale reading is charge the aircraft
// never drew. Two missed detail frames.
#define ZHIANN_CURRENT_FRESH_MS  1500

// Imbalance reporting, on SPREAD (max - min) rather than standard deviation.
// Standard deviation shrinks as packs are added, so a fixed threshold would
// mean something different on a two-pack flight than a five-pack one, and the
// value needed to reach any sane threshold cannot occur while the packs are
// electrically tied in parallel.
//
// The failure this must catch is a pack whose contactor has opened, which
// shows as its unloaded open-circuit voltage sitting above the loaded bus by
// around a volt and a half.
//
// NOT CALIBRATED UNDER LOAD. At rest four packs measured a 0.84 V spread,
// which is pure BMS measurement offset because no current is flowing, and
// 1.5 V is roughly twice that. Under load the healthy spread widens with each
// pack's internal resistance and nobody has measured by how much, so this may
// need raising. The current spread is the decisive in-flight check; at rest an
// open pack is undetectable by any means, since with no current it reads the
// same voltage as the bus.
//
// Current is only judged once the set is actually delivering, because sharing
// at idle is meaningless and the BMS reports exactly 0 A below a few amps.
#define ZHIANN_IMBALANCE_V_SPREAD    1.5f    // volts
#define ZHIANN_IMBALANCE_SOC_SPREAD  15.0f   // percent
#define ZHIANN_IMBALANCE_I_SPREAD    30.0f   // amps
#define ZHIANN_IMBALANCE_I_FLOOR     20.0f   // mean amps before current judged

const AP_Param::GroupInfo AP_BattMonitor_ZhiannBMS::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to the BMS reported current, to trim the nominal 1mA/LSB scale against a calibrated reference
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
AP_BattMonitor_ZhiannBMS *AP_BattMonitor_ZhiannBMS::_singleton;

AP_BattMonitor_ZhiannBMS::AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    // cells not yet received report as not-present
    memset(_state.cell_voltages.cells, 0xFF, sizeof(_state.cell_voltages.cells));
    for (uint8_t i = 0; i < ARRAY_SIZE(_nodes); i++) {
        memset(_nodes[i].cells, 0xFF, sizeof(_nodes[i].cells));
    }

    _state.healthy = false;

    // The whole pack set is one battery, so only the first instance of this
    // type does anything. A second one would decode the same frames twice and
    // publish a duplicate of the same battery.
    if (_singleton == nullptr) {
        _singleton = this;
        _can_driver = NEW_NOTHROW AP_BattMonitor_ZhiannBMS_CAN(*this);
        if (_can_driver == nullptr) {
            AP_BoardConfig::allocation_error("ZhiannBMS CAN driver");
        }
    }
}

// Decode one frame and file it against its node. Runs on the CAN driver
// thread. Every node is accepted: there is no mapping to get wrong.
bool AP_BattMonitor_ZhiannBMS::dispatch_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() ||
        frame.isErrorFrame()) {
        return false;
    }

    ZhiannBMS::Classified cls;
    if (!ZhiannBMS::classify(frame.id & AP_HAL::CANFrame::MaskExtID, cls)) {
        return false;
    }
    if (!ZhiannBMS::valid_dlc(cls.type, frame.dlc)) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    const uint32_t now_ms = AP_HAL::millis();

    // The alarm frame is anonymous: it carries no node, and applies to the
    // set. Union the bits; they are reported, and never gate health.
    if (cls.type == ZhiannBMS::FRAME_ALARM) {
        const uint16_t alarm = ZhiannBMS::u16(&frame.data[0]);
        const uint16_t warning = ZhiannBMS::u16(&frame.data[2]);
        if (!ZhiannBMS::fresh_ms(now_ms, _alarm_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            _alarm_bits = 0;
        }
        _alarm_bits |= alarm;
        _alarm_ms = now_ms;
        if (!ZhiannBMS::fresh_ms(now_ms, _warning_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            _warning_bits = 0;
        }
        _warning_bits |= warning;
        _warning_ms = now_ms;
        return true;
    }

    if (cls.node > ZhiannBMS::MAX_NODE) {
        return false;
    }
    Node &n = _nodes[cls.node];

    // fleet inventory: name each node once, and restart the settle window so
    // the pack count is not announced while packs are still arriving
    if (!(_nodes_announced & (1U << cls.node))) {
        _nodes_announced |= (1U << cls.node);
        _bus_settle_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ZhiannBMS: pack on node %u",
                      unsigned(cls.node));
    }
    n.last_ms = now_ms;

    handle_frame(n, cls.type, frame);
    return true;
}

// file one decoded frame into its node's state (CAN driver thread, _sem held)
void AP_BattMonitor_ZhiannBMS::handle_frame(Node &n, uint8_t frame_type,
                                            const AP_HAL::CANFrame &frame)
{
    const uint8_t *d = frame.data;
    const uint32_t now_ms = AP_HAL::millis();

    switch (frame_type) {

    case ZhiannBMS::FRAME_PACK_VOLT: {
        const float voltage = ZhiannBMS::pack_voltage(d);
        if (ZhiannBMS::pack_voltage_valid(voltage)) {
            n.voltage = voltage;
            n.detail_ms = now_ms;
        }
        const float current = ZhiannBMS::current_amps(d) * _curr_mult_cached;
        if (ZhiannBMS::current_valid(current)) {
            n.current = current;
        }
        break;
    }

    case ZhiannBMS::FRAME_TEMP_SOC: {
        float temperature;
        if (ZhiannBMS::select_temperature(ZhiannBMS::temp1_c(d),
                                          ZhiannBMS::temp2_c(d),
                                          temperature)) {
            n.temperature = temperature;
            n.temp_ms = now_ms;
        }
        if (ZhiannBMS::soc_fine_valid(d)) {
            n.soc_tenths = ZhiannBMS::soc_fine_tenths(d);
            n.soc_ms = now_ms;
        }
        break;
    }

    case ZhiannBMS::FRAME_SOC_COARSE:
        // Cadence of this frame doubles when two packs share the node, which
        // is how a collision is spotted. Detection only - it is information.
        n.dup.feed(now_ms);
        if (ZhiannBMS::soc_coarse_valid(d) &&
            !ZhiannBMS::fresh_ms(now_ms, n.soc_ms, ZHIANN_FINE_SOC_STALE_MS)) {
            n.soc_tenths = uint16_t(ZhiannBMS::soc_coarse_pct(d)) * 10;
            n.soc_ms = now_ms;
        }
        break;

    case ZhiannBMS::FRAME_PACK_ID:
        n.pack_id = ZhiannBMS::pack_id(d);
        n.identity.feed(n.pack_id, now_ms);
        break;

    default:
        // cell slices: publish only complete, atomically assembled sets
        if (frame_type == ZhiannBMS::FRAME_CELL_1_2) {
            const uint16_t reported = ZhiannBMS::u16(&d[2]);
            n.cell_count = reported > 24 ? 24 : uint8_t(reported);
        }
        if (n.accumulator.feed(frame_type, d, now_ms,
                               ZHIANN_CELL_SNAPSHOT_SPAN_MS)) {
            const uint16_t *cells = n.accumulator.cells();
            for (uint8_t i = 0; i < ARRAY_SIZE(n.cells); i++) {
                n.cells[i] = cells[i];
            }
            n.cells_ms = now_ms;
            n.accumulator.reset();
        }
        break;
    }
}

// Reduce every fresh node to one battery. Returns how many packs contributed.
uint8_t AP_BattMonitor_ZhiannBMS::aggregate(uint32_t now_ms, uint64_t now_us)
{
    // Gathered first and reduced second: the imbalance test needs the extremes
    // anyway, and a running sum of squares would lose the spread to float
    // cancellation at ~102V (see ZhiannBMS::reduce).
    float v[ARRAY_SIZE(_nodes)], amps[ARRAY_SIZE(_nodes)], soc[ARRAY_SIZE(_nodes)];
    uint32_t cell_sum[24] {};
    uint8_t live = 0, n_current = 0, n_soc = 0, with_cells = 0, with_temp = 0;
    float t_max = 0;
    _dup_any = false;

    for (uint8_t node = 0; node < ARRAY_SIZE(_nodes); node++) {
        Node &n = _nodes[node];
        n.contributing = false;
        if (!ZhiannBMS::fresh_ms(now_ms, n.last_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            continue;
        }
        if (n.dup.active(now_ms) || n.identity.active(now_ms)) {
            _dup_any = true;
        }
        // present but not contributing: a pack in standby broadcasts SOC only
        if (!ZhiannBMS::fresh_ms(now_ms, n.detail_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            continue;
        }
        n.contributing = true;
        v[live++] = n.voltage;

        // Current is summed and then integrated, so a stale value is charge
        // the aircraft never drew. Hold it to a much tighter window than
        // membership: a pack sends this frame every ~508ms, so 1.5s is two
        // missed frames. Beyond that the pack drops out of the total rather
        // than contributing a reading that is no longer true.
        if (ZhiannBMS::fresh_ms(now_ms, n.detail_ms, ZHIANN_CURRENT_FRESH_MS)) {
            amps[n_current++] = n.current;
        }
        if (ZhiannBMS::fresh_ms(now_ms, n.soc_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            soc[n_soc++] = n.soc_tenths * 0.1f;
        }
        if (ZhiannBMS::fresh_ms(now_ms, n.temp_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            if (with_temp == 0 || n.temperature > t_max) {
                t_max = n.temperature;
            }
            with_temp++;
        }
        if (n.cell_count == 24 &&
            ZhiannBMS::fresh_ms(now_ms, n.cells_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            for (uint8_t c = 0; c < 24; c++) {
                cell_sum[c] += n.cells[c];
            }
            with_cells++;
        }
    }

    if (live == 0) {
        // no pack delivering: report zero rather than freezing the last
        // reading. 0V is ArduPilot's no-reading convention and cannot trip
        // the voltage failsafes, which are guarded by voltage > 0.
        _state.healthy = false;
        _state.voltage = 0;
        _state.current_amps = 0;
        _has_cell_voltages = false;
        _has_temperature = false;
        _soc_valid = false;
        // _has_current is deliberately left alone: it says the pack set is
        // capable of reporting current, not that a reading is available now.
        _sd_voltage = _sd_current = _sd_soc = 0;
        _spread_voltage = _spread_current = _spread_soc = 0;
        _mean_current = 0;
        memset(_state.cell_voltages.cells, 0xFF,
               sizeof(_state.cell_voltages.cells));
        // force the next sample to re-seed rather than integrate the outage
        _consumed_us = 0;
        return 0;
    }

    float mean_v, mean_i, mean_soc;
    ZhiannBMS::reduce(v, live, mean_v, _spread_voltage, _sd_voltage);
    ZhiannBMS::reduce(amps, n_current, mean_i, _spread_current, _sd_current);
    ZhiannBMS::reduce(soc, n_soc, mean_soc, _spread_soc, _sd_soc);

    _state.voltage = mean_v;
    // parallel packs: the set draws the sum of what its packs report
    _state.current_amps = mean_i * n_current;
    _mean_current = mean_i;

    // Averaged over the packs that actually reported it, not over the whole
    // set: dividing by the wrong count reports a pack set emptier than it is
    // and fakes a large spread at the same time.
    _soc_valid = n_soc > 0;
    if (_soc_valid) {
        _soc_pct = uint8_t(constrain_float(mean_soc + 0.5f, 0, 100));
    }

    _has_temperature = with_temp > 0;
    if (_has_temperature) {
        _state.temperature = t_max;
        _state.temperature_time = now_ms;
    }

    _has_cell_voltages = with_cells > 0;
    if (_has_cell_voltages) {
        for (uint8_t c = 0; c < ARRAY_SIZE(_state.cell_voltages.cells); c++) {
            _state.cell_voltages.cells[c] = uint16_t(cell_sum[c] / with_cells);
        }
    } else {
        memset(_state.cell_voltages.cells, 0xFF,
               sizeof(_state.cell_voltages.cells));
    }

    // consumption from the summed current, integrated once for the set
    if (n_current > 0) {
        _has_current = true;
        if (_consumed_us != 0) {
            const uint32_t dt_us =
                ZhiannBMS::consumption_dt_us(now_us - _consumed_us);
            if (dt_us > 0) {
                _state.last_time_micros = uint32_t(now_us);
                update_consumed(_state, dt_us);
            }
        }
        _consumed_us = now_us;
    }

    _state.healthy = true;
    return live;
}

void AP_BattMonitor_ZhiannBMS::read()
{
    if (_singleton != this) {
        // A second monitor of this type would publish a duplicate of the same
        // battery. Keep it unhealthy rather than silently wrong - but say so,
        // because ArduPilot's own prearm text is only "Battery N unhealthy"
        // and an operator upgrading from the old one-instance-per-pack
        // parameter set has no way to work out what is being complained about.
        _state.healthy = false;
        const uint32_t now_ms = AP_HAL::millis();
        if (_dup_warn_ms == 0 || now_ms - _dup_warn_ms >= 30000) {
            _dup_warn_ms = now_ms;
            char pfx[3] {};
            if (_state.instance > 0) {
                hal.util->snprintf(pfx, sizeof(pfx), "%X",
                                   unsigned(_state.instance + 1));
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ZhiannBMS: set BATT%s_MONITOR=0, one instance only",
                          pfx);
        }
        return;
    }

    WITH_SEMAPHORE(_sem);
    const uint32_t now_ms = AP_HAL::millis();
    const uint64_t now_us = AP_HAL::micros64();

    // snapshot for the CAN thread, which decodes current as frames arrive
    _curr_mult_cached = _curr_mult.get();

    // expire alarm telemetry rather than latching it
    if (_alarm_ms != 0 &&
        !ZhiannBMS::fresh_ms(now_ms, _alarm_ms, ZHIANN_NODE_TIMEOUT_MS)) {
        _alarm_bits = 0;
        _alarm_ms = 0;
    }
    if (_warning_ms != 0 &&
        !ZhiannBMS::fresh_ms(now_ms, _warning_ms, ZHIANN_NODE_TIMEOUT_MS)) {
        _warning_bits = 0;
        _warning_ms = 0;
    }

    uint32_t faults = 0;
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
    _fault_bitmask = faults;

    // alarms are messages only, per operator ruling: they never gate health
    if (a != 0 && (_alarm_gcs_ms == 0 || now_ms - _alarm_gcs_ms >= 10000)) {
        _alarm_gcs_ms = now_ms;
        char names[40] {};
        uint8_t used = 0;
        uint16_t unnamed = a;
        for (const auto &an : zhiann_alarm_names) {
            if (!(a & an.mask)) {
                continue;
            }
            unnamed &= ~an.mask;
            const int w = hal.util->snprintf(&names[used], sizeof(names) - used,
                                             "%s%s", used ? ", " : "", an.name);
            if (w <= 0 || uint32_t(used) + uint32_t(w) >= sizeof(names)) {
                used = sizeof(names) - 1;
                break;
            }
            used += uint8_t(w);
        }
        if (unnamed != 0 && used < sizeof(names) - 1) {
            hal.util->snprintf(&names[used], sizeof(names) - used, "%s0x%X",
                               used ? ", " : "", unsigned(unnamed));
        }
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ZhiannBMS: BMS alarm: %s", names);
    }

    const uint8_t live = aggregate(now_ms, now_us);

    report_inventory(now_ms, live);
    report_imbalance(now_ms);
    log_zbms(now_ms, live);
}

// Information only. The operator needs to know how many packs the aircraft
// found and whether any node looks like two packs; neither gates arming,
// because checking the count against what was loaded is a preflight step.
void AP_BattMonitor_ZhiannBMS::report_inventory(uint32_t now_ms, uint8_t live)
{
    const bool settled = _bus_settle_ms != 0 &&
                         now_ms - _bus_settle_ms >= ZHIANN_BUS_SETTLE_MS;
    if (!settled) {
        return;
    }

    if (!_inventory_done) {
        _inventory_done = true;
        _expected_packs = live;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ZhiannBMS: %u pack%s delivering",
                      unsigned(live), live == 1 ? "" : "s");
    }

    // A collision means one node is carrying two packs' frames, so the pack
    // count reads low. Say so, so the number above is not mistaken for a
    // missing pack.
    if (_dup_any && (_dup_warn_ms == 0 || now_ms - _dup_warn_ms >= 30000)) {
        _dup_warn_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ZhiannBMS: a node carries 2 packs, count reads low");
    }

    // Losing a pack in flight leaves the rest carrying its share.
    if (live < _expected_packs &&
        (_lost_warn_ms == 0 || now_ms - _lost_warn_ms >= 30000)) {
        _lost_warn_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ZhiannBMS: %u of %u packs, lost %u",
                      unsigned(live), unsigned(_expected_packs),
                      unsigned(_expected_packs - live));
    }
}

// One pack behaving unlike the others is the failure the averaging would
// otherwise hide, so measure the spread and say when it stops being normal.
void AP_BattMonitor_ZhiannBMS::report_imbalance(uint32_t now_ms)
{
    const char *what = nullptr;
    float value = 0;

    if (_spread_voltage > ZHIANN_IMBALANCE_V_SPREAD) {
        what = "V";
        value = _spread_voltage;
    } else if (_spread_soc > ZHIANN_IMBALANCE_SOC_SPREAD) {
        what = "SOC";
        value = _spread_soc;
    } else if (fabsf(_mean_current) > ZHIANN_IMBALANCE_I_FLOOR &&
               _spread_current > ZHIANN_IMBALANCE_I_SPREAD) {
        what = "A";
        value = _spread_current;
    }
    if (what == nullptr) {
        return;
    }
    if (_imbalance_warn_ms != 0 && now_ms - _imbalance_warn_ms < 30000) {
        return;
    }
    _imbalance_warn_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "ZhiannBMS: packs differ by %.1f %s", (double)value, what);
}

bool AP_BattMonitor_ZhiannBMS::capacity_remaining_pct(uint8_t &percentage) const
{
    if (_state.healthy && _soc_valid) {
        percentage = _soc_pct;
        return true;
    }
    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

// ZBMS at 2Hz: the aggregate plus the spread that produced it, then one row
// per live node so a single misbehaving pack can be found after the fact.
void AP_BattMonitor_ZhiannBMS::log_zbms(uint32_t now_ms, uint8_t live)
{
#if HAL_LOGGING_ENABLED
    if (now_ms - _last_log_ms < 500) {
        return;
    }
    _last_log_ms = now_ms;
    const uint64_t now_us = AP_HAL::micros64();

    // @LoggerMessage: ZBMS
    // @Description: Zhiann BMS pack set, aggregated as one battery
    // @Field: TimeUS: Time since system startup
    // @Field: Np: Packs delivering data
    // @Field: Dup: A node appears to carry two packs
    // @Field: Volt: Mean pack voltage
    // @Field: Curr: Total current, discharge positive
    // @Field: SOC: Mean state of charge
    // @Field: Temp: Highest pack temperature
    // @Field: SDV: Standard deviation of pack voltage
    // @Field: SDA: Standard deviation of pack current
    // @Field: SDS: Standard deviation of pack state of charge
    // @Field: Alm: Vendor alarm bitmask, unioned
    // @Field: Warn: Vendor warning bitmask, unioned
    AP::logger().WriteStreaming(
        "ZBMS", "TimeUS,Np,Dup,Volt,Curr,SOC,Temp,DV,DA,DS,SDV,SDA,SDS,Alm,Warn",
        "s#-vA%OvA-vA---", "F00000000000000", "QBBffBfffffffHH",
        now_us,
        live,
        uint8_t(_dup_any ? 1 : 0),
        (double)_state.voltage,
        (double)_state.current_amps,
        _soc_pct,
        (double)_state.temperature,
        (double)_spread_voltage,
        (double)_spread_current,
        (double)_spread_soc,
        (double)_sd_voltage,
        (double)_sd_current,
        (double)_sd_soc,
        _alarm_bits,
        _warning_bits);

    // @LoggerMessage: ZBND
    // @Description: Zhiann BMS per-node readings behind the aggregate
    // @Field: TimeUS: Time since system startup
    // @Field: Node: Proprietary broadcast node
    // @Field: Volt: Pack voltage
    // @Field: Curr: Pack current, discharge positive
    // @Field: SOC: Pack state of charge
    // @Field: Temp: Pack temperature
    // @Field: Dup: This node appears to carry two packs
    // @Field: ID: Per-pack identity from the 2s frame
    for (uint8_t node = 0; node < ARRAY_SIZE(_nodes); node++) {
        Node &n = _nodes[node];
        if (!ZhiannBMS::fresh_ms(now_ms, n.last_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            continue;
        }
        AP::logger().WriteStreaming(
            "ZBND", "TimeUS,Node,Ctb,Volt,Curr,SOC,Temp,Dup,ID",
            "s#-vA%O#-", "F00000000", "QBBffBfBI",
            now_us,
            node,
            uint8_t(n.contributing ? 1 : 0),
            (double)n.voltage,
            (double)n.current,
            uint8_t(n.soc_tenths / 10),
            (double)n.temperature,
            uint8_t(n.dup.active(now_ms) ? 1 : 0),
            n.pack_id);

        // Full cell set per pack. MAVLink carries only 14 cells and the
        // aggregate averages them away, so this is the only place a single
        // failing cell in a single pack can be seen after the fact.
        if (n.cell_count != 24 ||
            !ZhiannBMS::fresh_ms(now_ms, n.cells_ms, ZHIANN_NODE_TIMEOUT_MS)) {
            continue;
        }
        // @LoggerMessage: ZBC1
        // @Description: Zhiann BMS cells 1-12 for one pack
        // @Field: TimeUS: Time since system startup
        // @Field: Node: Proprietary broadcast node
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
            "ZBC1", "TimeUS,Node,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12",
            "s#vvvvvvvvvvvv", "F0CCCCCCCCCCCC", "QBHHHHHHHHHHHH",
            now_us, node,
            n.cells[0], n.cells[1], n.cells[2], n.cells[3],
            n.cells[4], n.cells[5], n.cells[6], n.cells[7],
            n.cells[8], n.cells[9], n.cells[10], n.cells[11]);

        // @LoggerMessage: ZBC2
        // @Description: Zhiann BMS cells 13-24 for one pack
        // @Field: TimeUS: Time since system startup
        // @Field: Node: Proprietary broadcast node
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
            "ZBC2", "TimeUS,Node,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23,V24",
            "s#vvvvvvvvvvvv", "F0CCCCCCCCCCCC", "QBHHHHHHHHHHHH",
            now_us, node,
            n.cells[12], n.cells[13], n.cells[14], n.cells[15],
            n.cells[16], n.cells[17], n.cells[18], n.cells[19],
            n.cells[20], n.cells[21], n.cells[22], n.cells[23]);
    }
#endif  // HAL_LOGGING_ENABLED
}

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
