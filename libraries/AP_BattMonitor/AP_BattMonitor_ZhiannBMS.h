#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

class AP_BattMonitor_ZhiannBMS : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    void read() override;

    bool has_current() const override { return _has_current; }
    bool has_consumed_energy() const override { return _has_current; }
    bool has_cell_voltages() const override { return _has_cell_voltages; }
    bool has_temperature() const override { return _has_temperature; }

    // BMS-reported SOC when available, else base-class coulomb count
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    // faults decoded from the BMS alarm frame (PF 0x24)
    uint32_t get_mavlink_fault_bitmask() const override { return _fault_bitmask; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // single shared MultiCAN callback: decodes the frame and dispatches
    // to the owning instance deterministically (in BATTn instance order)
    bool dispatch_frame(AP_HAL::CANFrame &frame);

    // returns true if this instance is bound to the given pack node
    bool matches_node(uint8_t node) const;

    // returns true if any instance explicitly selects this node via
    // BATTn_SERIAL_NUM
    static bool node_claimed(uint8_t node);

    // process one frame from this instance's pack (CAN driver thread)
    void handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame);

    void store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells);

    // all instances share one MultiCAN (registered by the first instance)
    static MultiCAN *_multican;
    static AP_BattMonitor_ZhiannBMS *_instances[AP_BATT_MONITOR_MAX_INSTANCES];
    static uint8_t _num_instances;

    AP_Float _curr_mult;

    HAL_Semaphore _sem;

    // pack node this instance is bound to when BATTn_SERIAL_NUM is -1;
    // only accessed from the CAN dispatch thread
    int8_t _auto_node = -1;

    // state accumulated from CAN frames under _sem, copied into _state
    // by read()
    AP_BattMonitor::BattMonitor_State _interim_state {};
    uint64_t _last_frame_us;        // 64-bit: liveness check cannot wrap
    uint32_t _fine_soc_ms;          // last 0.1%-resolution SOC frame
    uint32_t _alarm_ms;             // last alarm frame (PF 0x24)
    uint16_t _alarm_bits;
    uint16_t _warning_bits;
    // duplicate-node detection: a lone pack sends its status frame every
    // ~500ms; sub-300ms arrivals mean two packs share this node
    uint32_t _last_volt_ms;
    uint8_t _dup_score;
    uint8_t _soc_pct;
    bool _soc_valid;
    bool _cells_seen;
    bool _current_seen;

    // copies published by read() for lock-free main-thread accessors
    uint32_t _fault_bitmask;
    uint32_t _dup_warn_ms;
    bool _dup_active;
    uint8_t _soc_pct_pub;
    bool _soc_valid_pub;
    bool _has_current;
    bool _has_cell_voltages;
    bool _has_temperature;
};

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
