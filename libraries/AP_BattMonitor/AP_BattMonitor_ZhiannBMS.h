#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include "AP_BattMonitor_Backend.h"
#include "AP_BattMonitor_ZhiannBMS_decode.h"
#include <AP_CANManager/AP_CANSensor.h>

class AP_BattMonitor_ZhiannBMS_CAN;

class AP_BattMonitor_ZhiannBMS : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    void init() override;
    void read() override;

    bool has_current() const override { return _has_current; }
    bool has_consumed_energy() const override { return _has_current; }
    bool has_cell_voltages() const override { return _has_cell_voltages; }
    bool has_temperature() const override { return _has_temperature; }

    // BMS-reported SOC when the complete monitor state is healthy
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    // faults decoded from the BMS alarm frame (PF 0x24)
    uint32_t get_mavlink_fault_bitmask() const override { return _fault_bitmask; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    friend class AP_BattMonitor_ZhiannBMS_CAN;

    // single dedicated CAN callback: decodes the frame and dispatches
    // to the owning instance deterministically (in BATTn instance order)
    bool dispatch_frame(AP_HAL::CANFrame &frame);

    // returns true if this instance is bound to the given pack node
    bool matches_node(uint8_t node) const;

    // node this instance is bound to, for operator messages
    int8_t bound_node() const {
        return _configured_node >= 0 ? _configured_node :
               (_configured_node == -1 ? _auto_node : -1);
    }

    // returns true if any instance explicitly selects this node via
    // BATTn_SERIAL_NUM
    static bool node_claimed(uint8_t node);

    // process one frame from this instance's pack (CAN driver thread)
    void handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame);

    // Raise the consumed floor when BMS SOC implies more use than the
    // current integral, without ever erasing calibrated-current integration.
    void reconcile_consumption(uint16_t soc_tenths);

    // one-time GCS audit of static configuration errors, from the first
    // read() call of any instance
    static void announce_misconfiguration();

    // mark all configured instances unsafe while an extra physical pack is
    // broadcasting without a battery-monitor mapping
    void note_unmapped(uint32_t now_ms);

    // ZBMS/ZBC1/ZBC2 dataflash messages, 2Hz per instance from read()
    void log_zbms();

    // all instances share one dedicated CAN sensor (registered by the first)
    static AP_BattMonitor_ZhiannBMS_CAN *_can_driver;
    static AP_BattMonitor_ZhiannBMS *_instances[AP_BATT_MONITOR_MAX_INSTANCES];
    static uint8_t _num_instances;
    static uint16_t _nodes_announced;   // fleet inventory: one GCS info per node
    static uint32_t _unmapped_warn_ms[ZhiannBMS::MAX_NODE + 1];
    static bool _misconfig_checked;     // one-time configuration audit done

    AP_Float _curr_mult;

    HAL_Semaphore _sem;

    // pack node this instance is bound to when BATTn_SERIAL_NUM is -1;
    // only accessed from the CAN dispatch thread
    int8_t _auto_node = -1;
    // immutable snapshot taken after dynamic parameters have loaded:
    // -1 auto, 0..15 explicit, -2 invalid configuration
    int8_t _configured_node = -2;

    // state accumulated from CAN frames under _sem, copied into _state
    // by read()
    AP_BattMonitor::BattMonitor_State _interim_state {};
    uint64_t _last_frame_us = 0;    // 64-bit: liveness check cannot wrap
    uint32_t _last_soc_ms = 0;      // last SOC frame of any kind: still set
                                    // in standby, when detail frames stop
    uint32_t _soc_valid_ms = 0;     // last frame containing a valid 0..100 SOC
    uint32_t _fine_soc_ms = 0;      // last 0.1%-resolution SOC frame
    uint16_t _cells24[24];          // full cell set for ZBC1/ZBC2 logging
                                    // (the state array caps at 12/14)
    ZhiannBMS::CellAccumulator _cell_accumulator;
    float _cell_accumulator_voltage = 0;
    uint32_t _cell_snapshot_ms = 0;
    bool _cell_snapshot_coherent = false;
    ZhiannBMS::CoherenceDetector _coherence_detector;
    uint16_t _soc_frame_vmir = 0;   // voltage mirror from the SOC frame
    float _temp1_c = 0;
    float _temp2_c = 0;             // both sensors (state gets the max)
    uint8_t _cell_count = 0;        // as reported in frame +0x02
    uint32_t _alarm_ms = 0;         // last alarm frame (PF 0x24)
    uint32_t _warning_ms = 0;
    uint16_t _alarm_bits = 0;
    uint16_t _warning_bits = 0;
    uint32_t _current_warn_ms = 0;  // rate limit: implausible current
    uint32_t _temp_warn_ms = 0;     // rate limit: both temp sensors bad
    uint32_t _cellcount_warn_ms = 0; // rate limit: cell count != 24
    // duplicate-node detection over PACK_VOLT frame arrival intervals
    ZhiannBMS::DupDetector _dup;
    uint8_t _soc_pct = 0;
    bool _soc_valid = false;
    bool _current_seen = false;
    bool _current_fault = false;    // last PACK current was implausible

    // copies published by read() for lock-free main-thread accessors
    uint32_t _fault_bitmask = 0;
    uint32_t _dup_warn_ms = 0;
    uint32_t _standby_warn_ms = 0;
    uint32_t _unmapped_ms = 0;
    uint32_t _coherence_warn_ms = 0;
    uint32_t _alarm_gcs_ms = 0;     // rate limit: active-alarm operator message
    uint32_t _last_log_ms = 0;
    bool _dup_active = false;
    bool _standby = false;
    bool _unmapped_active = false;
    bool _coherence_fault = false;
    uint8_t _soc_pct_pub = 0;
    bool _soc_valid_pub = false;
    bool _has_current = false;
    bool _has_cell_voltages = false;
    bool _has_temperature = false;
};

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
