#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include "AP_BattMonitor_Backend.h"
#include "AP_BattMonitor_ZhiannBMS_decode.h"
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

    // node this instance is bound to, for operator messages
    int8_t bound_node() const {
        const int32_t serial = _params._serial_number.get();
        return serial >= 0 ? (int8_t)serial : _auto_node;
    }

    // returns true if any instance explicitly selects this node via
    // BATTn_SERIAL_NUM
    static bool node_claimed(uint8_t node);

    // process one frame from this instance's pack (CAN driver thread)
    void handle_pack_frame(uint8_t frame_type, AP_HAL::CANFrame &frame);

    void store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells);

    // ZBMS/ZBC1/ZBC2 dataflash messages, 2Hz per instance from read()
    void log_zbms();

    // all instances share one MultiCAN (registered by the first instance)
    static MultiCAN *_multican;
    static AP_BattMonitor_ZhiannBMS *_instances[AP_BATT_MONITOR_MAX_INSTANCES];
    static uint8_t _num_instances;
    static uint16_t _nodes_announced;   // fleet inventory: one GCS info per node
    static uint32_t _unmapped_warn_ms;  // rate limit for unmapped-node warning

    AP_Float _curr_mult;

    HAL_Semaphore _sem;

    // pack node this instance is bound to when BATTn_SERIAL_NUM is -1;
    // only accessed from the CAN dispatch thread
    int8_t _auto_node = -1;

    // state accumulated from CAN frames under _sem, copied into _state
    // by read()
    AP_BattMonitor::BattMonitor_State _interim_state {};
    uint64_t _last_frame_us;        // 64-bit: liveness check cannot wrap
    uint32_t _last_soc_ms;          // last SOC frame of any kind: still set
                                    // in standby, when detail frames stop
    uint32_t _fine_soc_ms;          // last 0.1%-resolution SOC frame
    uint16_t _cells24[24];          // full cell set for ZBC1/ZBC2 logging
                                    // (the state array caps at 12/14)
    uint16_t _soc_frame_vmir;       // voltage mirror from the SOC frame
    float _temp1_c, _temp2_c;       // both sensors (state gets the max)
    uint8_t _cell_count;            // as reported in frame +0x02
    uint32_t _alarm_ms;             // last alarm frame (PF 0x24)
    uint16_t _alarm_bits;
    uint16_t _warning_bits;
    // duplicate-node detection over PACK_VOLT frame arrival intervals
    ZhiannBMS::DupDetector _dup;
    uint8_t _soc_pct;
    bool _soc_valid;
    bool _cells_seen;
    bool _current_seen;

    // copies published by read() for lock-free main-thread accessors
    uint32_t _fault_bitmask;
    uint32_t _dup_warn_ms;
    uint32_t _standby_warn_ms;
    uint32_t _last_log_ms;
    bool _dup_active;
    bool _standby;
    uint8_t _soc_pct_pub;
    bool _soc_valid_pub;
    bool _has_current;
    bool _has_cell_voltages;
    bool _has_temperature;
};

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
