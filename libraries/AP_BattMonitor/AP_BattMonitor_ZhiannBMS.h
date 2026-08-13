#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include "AP_BattMonitor_Backend.h"
#include "AP_BattMonitor_ZhiannBMS_decode.h"
#include <AP_CANManager/AP_CANSensor.h>

class AP_BattMonitor_ZhiannBMS_CAN;

// One battery monitor for the whole Zhiann pack set.
//
// The packs are wired in parallel and each broadcasts on its own bus "node",
// a number the pack firmware picks for itself and which can collide with
// another pack's. Rather than map nodes to battery instances - which turns
// every node quirk into an operational problem - this backend consumes every
// node and publishes the set as a single battery: mean voltage, summed
// current, mean state of charge, highest temperature.
//
// Node count and suspected collisions are reported to the operator as
// information only and never gate health. What gates health is having at
// least one pack delivering complete, fresh data.
class AP_BattMonitor_ZhiannBMS : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    void init() override {}
    void read() override;

    bool has_current() const override { return _has_current; }
    bool has_consumed_energy() const override { return _has_current; }
    bool has_cell_voltages() const override { return _has_cell_voltages; }
    bool has_temperature() const override { return _has_temperature; }

    // BMS-reported SOC, averaged across the packs
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    // faults decoded from the BMS alarm frame (PF 0x24)
    uint32_t get_mavlink_fault_bitmask() const override { return _fault_bitmask; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    friend class AP_BattMonitor_ZhiannBMS_CAN;

    // everything one pack has told us; one of these per bus node
    struct Node {
        uint32_t last_ms;        // any frame: liveness
        uint32_t detail_ms;      // pack voltage/current frame
        uint32_t soc_ms;         // last valid state of charge
        uint32_t temp_ms;
        uint32_t cells_ms;       // last complete 24-cell set
        float    voltage;
        float    current;
        float    temperature;    // higher of the pack's two sensors
        uint16_t soc_tenths;
        uint16_t cells[24];
        uint8_t  cell_count;     // as reported by the pack
        uint32_t pack_id;        // stable per-pack id from the 2s frame
        ZhiannBMS::CellAccumulator accumulator;
        ZhiannBMS::SocCadenceDupDetector dup;
        ZhiannBMS::IdentityDupDetector identity;
        bool     seen;           // ever heard from, this boot
    };

    // decode one frame and file it against its node (CAN driver thread)
    bool dispatch_frame(AP_HAL::CANFrame &frame);
    void handle_frame(Node &n, uint8_t node_num, uint8_t frame_type,
                      const AP_HAL::CANFrame &frame);

    // aggregate every fresh node into _state; returns packs contributing
    uint8_t aggregate(uint32_t now_ms, uint64_t now_us);

    // information-only operator messages
    void report_inventory(uint32_t now_ms, uint8_t live);
    void report_imbalance(uint32_t now_ms);

    // ZBMS/ZBC1/ZBC2 dataflash messages
    void log_zbms(uint32_t now_ms, uint8_t live);

    // one shared CAN sensor, registered by the first instance
    static AP_BattMonitor_ZhiannBMS_CAN *_can_driver;
    static AP_BattMonitor_ZhiannBMS *_singleton;

    AP_Float _curr_mult;

    HAL_Semaphore _sem;

    Node _nodes[ZhiannBMS::MAX_NODE + 1] {};

    // alarm and warning words, unioned across the packs (they are anonymous)
    uint32_t _alarm_ms = 0;
    uint32_t _warning_ms = 0;
    uint16_t _alarm_bits = 0;
    uint16_t _warning_bits = 0;

    // consumption is integrated once, from the summed pack current
    uint64_t _consumed_us = 0;

    // spread across the packs, recomputed each read()
    float _sd_voltage = 0;
    float _sd_current = 0;
    float _sd_soc = 0;
    float _mean_current = 0;

    // operator messaging state
    uint32_t _bus_settle_ms = 0;    // last time a new node appeared
    uint32_t _imbalance_warn_ms = 0;
    uint32_t _lost_warn_ms = 0;
    uint32_t _alarm_gcs_ms = 0;
    uint32_t _dup_warn_ms = 0;
    uint32_t _last_log_ms = 0;
    uint16_t _nodes_announced = 0;  // bitmask: one info message per node
    uint8_t  _expected_packs = 0;   // count at settle, to notice a loss
    bool     _inventory_done = false;

    // published copies for the lock-free accessors
    uint32_t _fault_bitmask = 0;
    uint8_t  _soc_pct = 0;
    bool     _soc_valid = false;
    bool     _has_current = false;
    bool     _has_cell_voltages = false;
    bool     _has_temperature = false;
    bool     _dup_any = false;
};

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
