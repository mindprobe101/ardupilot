#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

class AP_BattMonitor_ZhiannBMS : public CANSensor, public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    void read() override;

    // the current field of the protocol has not been identified yet
    bool has_current() const override { return false; }
    bool has_consumed_energy() const override { return false; }
    bool has_cell_voltages() const override { return _has_cell_voltages; }
    bool has_temperature() const override { return _has_temperature; }

    // SOC is reported directly by the BMS
    bool capacity_remaining_pct(uint8_t &percentage) const override;

private:
    // handler for incoming frames, runs in the CAN driver thread
    void handle_frame(AP_HAL::CANFrame &frame) override;

    void store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells);

    HAL_Semaphore _sem;

    // state accumulated from CAN frames, copied into _state by read()
    struct {
        float voltage;
        float temperature;
        uint32_t temperature_time_ms;
        uint32_t last_frame_us;
        uint16_t cells_mv[AP_BATT_MONITOR_CELLS_MAX];
        bool cells_seen;
        uint8_t soc_pct;
        bool soc_valid;
        bool soc_is_fine;   // SOC seen from the 0.1% resolution frame
    } _interim;

    bool _has_cell_voltages;
    bool _has_temperature;
};

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
